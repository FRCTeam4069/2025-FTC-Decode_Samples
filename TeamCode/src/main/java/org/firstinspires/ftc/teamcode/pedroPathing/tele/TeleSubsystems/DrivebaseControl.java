package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Clean DrivebaseControl — field-centric driving + slow mode only
 * No auto-centering, no override methods, no extra parameters
 */
public class DrivebaseControl {

    private final Follower follower;
    private final BNO055IMU imu;

    private double imuOffset = 0.0;
    private boolean slowModeEnabled = false;

    public DrivebaseControl(Follower follower, BNO055IMU imu) {
        this.follower = follower;
        this.imu = imu;
    }

    /**
     * Standard teleop drive — called every loop
     * @param y    forward/back (left stick Y)
     * @param x    strafe (left stick X)
     * @param rx   rotate (right stick X)
     * @param fieldCentric whether to use field-centric driving
     */
    public void updateDrive(double y, double x, double rx, boolean fieldCentric) {
        // Cubic response for smoother low-speed control
        y  = Math.pow(y,  3);
        x  = Math.pow(x,  3);
        rx = Math.pow(rx, 3);

        // Slow mode
        double maxPower = slowModeEnabled ? 0.25 : 1.0;

        double driveY = y;
        double driveX = x;

        // Field-centric transformation
        if (fieldCentric) {
            double heading = getHeading();
            driveX = x * Math.cos(heading) - y * Math.sin(heading);
            driveY = x * Math.sin(heading) + y * Math.cos(heading);
        }

        // Send to Pedro Pathing follower
        follower.setTeleOpDrive(
                driveY * maxPower,
                driveX * maxPower,
                rx * maxPower,
                true // Use TeleOp PID
        );
    }

    /** Toggle slow mode (usually on right bumper) */
    public void toggleSlowMode() {
        slowModeEnabled = !slowModeEnabled;
    }

    /** Reset field-centric heading to current direction (usually on X button) */
    public void resetHeading() {
        imuOffset = imu.getAngularOrientation().firstAngle;
    }

    /** Current robot heading in radians, with offset applied */
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle - imuOffset;
    }

    // Simple getters
    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

    public double getMaxPower() {
        return slowModeEnabled ? 0.25 : 1.0;
    }
}
