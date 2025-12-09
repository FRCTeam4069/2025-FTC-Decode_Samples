package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;

/**
 * SUPER SIMPLE auto-centering
 * Just runs PID to minimize heading error to AprilTag 20
 * No IMU needed - uses odometry heading from PoseEstimator
 */
public class AutoCenteringController {

    private final Follower follower;
    private final PoseEstimator poseEstimator;

    // PID state
    private double integralError = 0.0;
    private double lastError = 0.0;
    private long lastUpdateTime = 0;

    // Control state
    private boolean isActive = false;

    // PID gains (tune these!)
    private static final double KP = 0.001;
    private static final double KI = 0.0;
    private static final double KD = 0.002;
    private static final double TOLERANCE = Math.toRadians(2.0);  // 2 degrees

    public AutoCenteringController(Follower follower, PoseEstimator poseEstimator) {
        this.follower = follower;
        this.poseEstimator = poseEstimator;
        this.lastUpdateTime = System.nanoTime();
    }

    /**
     * Start auto-centering
     */
    public void startCentering() {
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
     * Calculate rotation correction
     * Returns rotation power (-1.0 to 1.0)
     */
    public double getRotationCorrection() {
        if (!isActive) {
            return 0.0;
        }

        // Check if we have valid tracking
        if (!poseEstimator.hasValidTracking()) {
            return 0.0;
        }

        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9;
        lastUpdateTime = currentTime;

        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        // Get heading error
        // Positive = need to turn LEFT
        // Negative = need to turn RIGHT
        double error = poseEstimator.getHeadingErrorToTag();

        // PID calculation
        double p = error * KP;

        integralError += error * dt;
        integralError = Math.max(-10.0, Math.min(10.0, integralError));
        double i = integralError * KI;

        double d = 0.0;
        if (dt > 0) {
            d = ((error - lastError) / dt) * KD;
        }
        lastError = error;

        // Combine
        double correction = p + i + d;

        // Clamp to motor range
        correction = Math.max(-1.0, Math.min(1.0, correction));

        return correction;
    }

    /**
     * Check if centered
     */
    public boolean isCentered() {
        if (!isActive || !poseEstimator.hasValidTracking()) {
            return false;
        }

        double error = Math.abs(poseEstimator.getHeadingErrorToTag());
        return error < TOLERANCE;
    }

    /**
     * Check if active
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * Get heading error in degrees
     */
    public double getHeadingErrorDegrees() {
        return poseEstimator.getHeadingErrorDegrees();
    }
}