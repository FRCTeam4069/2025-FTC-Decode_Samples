package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * AprilTag centering using Limelight 3A.
 * ONLY rotates the robot to align the camera center with the AprilTag center.
 * Returns correction value to be added to manual drive inputs.
 */
public class AprilTagCentering {

    private final Follower follower;
    private final BNO055IMU imu;
    private final Limelight3A limelight;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    // Conversion: Limelight returns meters; convert to inches for distance tracking
    private static final double METERS_TO_INCHES = 39.37007874;

    // PD constants for rotation only
    private double kP_rotation = 0.05;   //0.0075  // Rotational proportional gain
    private double kD_rotation = 0.005;     // Rotational derivative gain

    // Tolerance for "centered" detection (horizontal only)
    private double angleTolerance = 2.0;     // degrees (tx tolerance)

    // Maximum rotation power during centering
    private double maxRotationPower = 0.666666666666;

    // State tracking
    private boolean wasCentered = false;
    private double lastErrorAngle = 0.0;
    private double imuOffset = 0.0;
    private double currentCorrectionRotation = 0.0;

    // Distance tracking (for interpolation compatibility)
    private double lastDistanceInches = 0.0;
    private boolean lastTagVisible = false;
    private double lastTx = 0.0;  // Horizontal offset from crosshair

    public AprilTagCentering(Follower follower, BNO055IMU imu, Limelight3A limelight,
                             Gamepad gamepad1, Gamepad gamepad2) {
        this.follower = follower;
        this.imu = imu;
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Update centering control - call this every loop
     * @param shouldCenter Whether centering is active (e.g. right trigger)
     * @param dt Delta time since last update (seconds) (for derivative calculations)
     * @return The rotation correction to add to manual drive (0 if not centering)
     */
    public double update(boolean shouldCenter, double dt) {
        // Always update distance and visibility regardless of centering mode
        updateDistanceTracking();

        if (!shouldCenter) {
            // Reset state when not centering
            wasCentered = false;
            lastErrorAngle = 0.0;
            currentCorrectionRotation = 0.0;
            return 0.0;
        }

        // Provide IMU yaw to Limelight for best 3D localization (degrees)
        double imuYawDegrees = getHeading();
        try {
            limelight.updateRobotOrientation(imuYawDegrees);
        } catch (Exception ignored) {
            // Some older SDKs/versions may not have this method; safe to ignore if absent.
        }

        LLResult result = limelight.getLatestResult();

        // Check if we have a valid target
        if (result == null || !result.isValid()) {
            // No target - no correction
            wasCentered = false;
            currentCorrectionRotation = 0.0;
            return 0.0;
        }

        // CRITICAL: Use tx (2D horizontal offset) directly from Limelight
        // This is the raw 2D crosshair offset, NOT derived from 3D pose
        // tx is in degrees: negative = target is left of crosshair, positive = target is right
        double tx = result.getTx();
        lastTx = tx;

        // Error is simply the horizontal offset
        // We want tx to be 0 (target centered in camera view)
        double errorAngle = tx;

        // Protect against dt <= 0
        if (dt <= 0) dt = 1e-3;

        // Calculate derivative (rate of change of error)
        double derivAngle = (errorAngle - lastErrorAngle) / dt;

        // PD control for rotation
        // Negative sign because: if target is to the right (positive tx), we need to rotate right (positive rotation)
        double correctionRotation = -(kP_rotation * errorAngle + kD_rotation * derivAngle);

        // Clamp to maximum rotation power
        correctionRotation = clamp(correctionRotation, -maxRotationPower, maxRotationPower);

        // Store the correction for external use
        currentCorrectionRotation = correctionRotation;

        // Check if centered within tolerance
        boolean isCentered = Math.abs(errorAngle) < angleTolerance;

        // Trigger vibration when first reaching centered position
        if (isCentered && !wasCentered) {
            try {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } catch (Exception ignored) {}
        }

        wasCentered = isCentered;
        lastErrorAngle = errorAngle;

        return correctionRotation;
    }

    /**
     * Update distance tracking from Limelight data
     * Uses 3D pose for distance calculation, but 2D tx for centering
     */
    private void updateDistanceTracking() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                lastTagVisible = true;

                // Get tx for tracking (this is 2D data, always accurate)
                lastTx = result.getTx();

                // Try to get 3D pose for distance calculation ONLY
                // We use 3D pose for distance but 2D tx for centering
                Pose3D botpose = null;
                try {
                    botpose = result.getBotpose_MT2();
                } catch (Exception ignored) {}

                if (botpose == null) {
                    try {
                        botpose = result.getBotpose();
                    } catch (Exception ignored) {}
                }

                if (botpose != null) {
                    // Calculate distance from origin (using X and Y position)
                    double x = botpose.getPosition().x * METERS_TO_INCHES;
                    double y = botpose.getPosition().y * METERS_TO_INCHES;
                    lastDistanceInches = Math.sqrt(x * x + y * y);
                } else {
                    // Fallback: use ty (vertical angle) to estimate distance if available
                    // This is less accurate but better than nothing
                    try {
                        double ty = result.getTy();
                        // Simple distance estimation (you may need to tune this for your setup)
                        // Assuming camera is mounted at a known height and angle
                        lastDistanceInches = 100.0; // Default fallback value
                    } catch (Exception ignored) {}
                }
            } else {
                lastTagVisible = false;
            }
        } catch (Exception e) {
            lastTagVisible = false;
        }
    }

    /**
     * Get the current rotation correction value
     */
    public double getCorrectionRotation() {
        return currentCorrectionRotation;
    }

    /**
     * Get distance to AprilTag in inches (for interpolation compatibility)
     */
    public double getDistanceInches() {
        return lastDistanceInches;
    }

    /**
     * Check if AprilTag is visible (for interpolation compatibility)
     */
    public boolean isTagVisible() {
        return lastTagVisible;
    }

    /**
     * Get current horizontal offset (tx) in degrees
     */
    public double getHorizontalOffset() {
        return lastTx;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Get current robot heading from IMU (degrees)
     */
    private double getHeading() {
        try {
            return imu.getAngularOrientation().firstAngle - imuOffset;
        } catch (Exception e) {
            return -imuOffset;
        }
    }

    /**
     * Reset heading offset
     */
    public void resetHeading() {
        try {
            imuOffset = imu.getAngularOrientation().firstAngle;
        } catch (Exception e) {
            imuOffset = 0.0;
        }
    }

    /**
     * Check if robot is currently centered within tolerance
     */
    public boolean isCentered() {
        return wasCentered;
    }

    // Tuning setters
    public void setRotationPID(double kP, double kD) {
        this.kP_rotation = kP;
        this.kD_rotation = kD;
    }

    public void setAngleTolerance(double tolerance) {
        this.angleTolerance = tolerance;
    }

    public void setMaxRotationPower(double power) {
        this.maxRotationPower = power;
    }
}