package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

/**
 * Handles RPM interpolation based on distance and shooting sequence logic
 */
public class ShooterInterpolation {

    private boolean isInterpolationShooting = false;
    private double interpolatedTargetRPM = 3500.0;
    private boolean servosLowered = false;
    private double servoLowerTime = 0.0;

    /**
     * Starts interpolation shooting sequence
     */
    public void startInterpolationSequence(double distanceInches) {
        isInterpolationShooting = true;
        servosLowered = false;

        if (distanceInches > 0) {
            interpolatedTargetRPM = interpolateRPM(distanceInches) + RobotConstants.rpm_offset;
        } else {
            interpolatedTargetRPM = 3500.0;
        }
    }

    /**
     * Stops interpolation shooting sequence
     */
    public void stopInterpolationSequence() {
        isInterpolationShooting = false;
        servosLowered = false;
    }

    /**
     * Updates servo lowering state when shooters reach target
     */
    public void setServosLowered(double currentTime) {
        servosLowered = true;
        servoLowerTime = currentTime;
    }

    /**
     * Checks if enough time has passed for push servos
     */
    public boolean shouldLowerPushServos(double currentTime) {
        return servosLowered && (currentTime - servoLowerTime >= RobotConstants.PUSH_SERVO_DELAY);
    }

    /**
     * Interpolates RPM based on distance using linear interpolation
     */
    private double interpolateRPM(double distanceInches) {
        // If distance is below minimum, use minimum RPM
        if (distanceInches <= RobotConstants.DISTANCE_DATA[0]) {
            return RobotConstants.RPM_DATA[0];
        }

        // If distance is above maximum, use maximum RPM
        if (distanceInches >= RobotConstants.DISTANCE_DATA[RobotConstants.DISTANCE_DATA.length - 1]) {
            return RobotConstants.RPM_DATA[RobotConstants.RPM_DATA.length - 1];
        }

        // Find the two data points to interpolate between
        for (int i = 0; i < RobotConstants.DISTANCE_DATA.length - 1; i++) {
            if (distanceInches >= RobotConstants.DISTANCE_DATA[i] &&
                    distanceInches <= RobotConstants.DISTANCE_DATA[i + 1]) {
                // Linear interpolation formula
                double x0 = RobotConstants.DISTANCE_DATA[i];
                double x1 = RobotConstants.DISTANCE_DATA[i + 1];
                double y0 = RobotConstants.RPM_DATA[i];
                double y1 = RobotConstants.RPM_DATA[i + 1];

                return y0 + (distanceInches - x0) * (y1 - y0) / (x1 - x0);
            }
        }

        // Fallback
        return 3500.0;
    }

    // Getters
    public boolean isInterpolationShooting() {
        return isInterpolationShooting;
    }

    public double getInterpolatedTargetRPM() {
        return interpolatedTargetRPM;
    }

    public boolean areServosLowered() {
        return servosLowered;
    }
}