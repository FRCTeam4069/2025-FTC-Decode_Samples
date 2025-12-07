package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Controls robot heading to center on goal using pose estimation
 * Works WITHOUT needing vision to be currently available
 */
public class AutoCenteringController {

    private final Follower follower;
    private final BNO055IMU imu;
    private final PoseEstimator poseEstimator;

    // PID state
    private double integralError = 0.0;
    private double lastError = 0.0;
    private long lastUpdateTime = 0;

    // Control state
    private boolean isActive = false;
    private int targetGoalId = -1;

    public AutoCenteringController(Follower follower, BNO055IMU imu, PoseEstimator poseEstimator) {
        this.follower = follower;
        this.imu = imu;
        this.poseEstimator = poseEstimator;
        this.lastUpdateTime = System.nanoTime();
    }

    /**
     * Start auto-centering on the CLOSEST goal
     */
    public void startCentering() {
        // Determine which goal is CLOSEST to the robot
        targetGoalId = poseEstimator.getClosestGoal();

        isActive = true;
        integralError = 0.0;
        lastError = 0.0;
        lastUpdateTime = System.nanoTime();
    }

    /**
     * Stop auto-centering
     */
    public void stopCentering() {
        isActive = false;
        integralError = 0.0;
        lastError = 0.0;
    }

    /**
     * Calculate rotation correction to center on goal
     * Call this every loop cycle when active
     *
     * @return rotation correction value (-1.0 to 1.0) for mecanum drive
     */
    public double getRotationCorrection() {
        if (!isActive || targetGoalId == -1) {
            return 0.0;
        }

        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
        lastUpdateTime = currentTime;

        if (dt <= 0 || dt > 1.0) { // Sanity check
            dt = 0.02; // Assume 50Hz if something weird happened
        }

        // Get current heading error (in radians)
        double headingError = getHeadingErrorToGoal();

        // PID calculation
        double proportional = headingError * FieldConstants.HEADING_KP;

        integralError += headingError * dt;
        // Anti-windup
        integralError = Math.max(-10.0, Math.min(10.0, integralError));
        double integral = integralError * FieldConstants.HEADING_KI;

        double derivative = 0.0;
        if (dt > 0) {
            derivative = ((headingError - lastError) / dt) * FieldConstants.HEADING_KD;
        }

        lastError = headingError;

        // Combine PID terms
        double correction = proportional + integral + derivative;

        // Clamp to [-1, 1] range for motor power
        correction = Math.max(-1.0, Math.min(1.0, correction));

        return correction;
    }

    /**
     * Check if we're centered on the goal (within tolerance)
     */
    public boolean isCentered() {
        if (!isActive || targetGoalId == -1) {
            return false;
        }

        double headingError = Math.abs(getHeadingErrorToGoal());
        return headingError < FieldConstants.HEADING_TOLERANCE;
    }

    /**
     * Get heading error to the target goal
     */
    private double getHeadingErrorToGoal() {
        if (targetGoalId == -1) {
            return 0.0;
        }

        // Get goal position
        double[] goalCenter = FieldConstants.getGoalCenterForTag(targetGoalId);
        double goalX = goalCenter[0];
        double goalY = goalCenter[1];

        // Get robot position
        double robotX = poseEstimator.getX();
        double robotY = poseEstimator.getY();
        double robotHeading = poseEstimator.getHeading();

        // Calculate desired heading to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double desiredHeading = Math.atan2(dy, dx);

        // Calculate error
        double error = normalizeAngle(desiredHeading - robotHeading);

        return error;
    }

    /**
     * Get the target goal ID we're centering on
     */
    public int getTargetGoalId() {
        return targetGoalId;
    }

    /**
     * Check if auto-centering is active
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * Get distance to target goal (for telemetry)
     */
    public double getDistanceToGoal() {
        if (targetGoalId == -1) {
            return -1.0;
        }

        double[] goalCenter = FieldConstants.getGoalCenterForTag(targetGoalId);
        double goalX = goalCenter[0];
        double goalY = goalCenter[1];

        double dx = goalX - poseEstimator.getX();
        double dy = goalY - poseEstimator.getY();

        return Math.hypot(dx, dy);
    }

    /**
     * Get current heading error in degrees (for telemetry)
     */
    public double getHeadingErrorDegrees() {
        return Math.toDegrees(getHeadingErrorToGoal());
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}