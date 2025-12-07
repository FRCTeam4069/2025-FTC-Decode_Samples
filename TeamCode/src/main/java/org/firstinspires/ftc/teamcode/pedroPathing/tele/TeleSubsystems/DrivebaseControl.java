package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Manages robot drivebase control including field-centric driving and slow mode
 */
public class DrivebaseControl {

    private Follower follower;
    private BNO055IMU imu;
    private double imuOffset = 0.0;
    private boolean slowModeEnabled = false;

    public DrivebaseControl(Follower follower, BNO055IMU imu) {
        this.follower = follower;
        this.imu = imu;
    }

    /**
     * Updates drivebase with driver inputs
     */
    public void updateDrive(double y, double x, double rx, boolean centeringActive) {
        // Apply cubic inputs for smooth control
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = Math.pow(rx, 3);

        // Apply slow mode scaling
        double maxPower = slowModeEnabled ? 0.25 : 1.0;

        // Field-centric rotation
        double heading = getHeading();
        double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

        // Set drive powers
        follower.setTeleOpDrive(rotatedY * maxPower, rotatedX * maxPower, rx * maxPower, !centeringActive);
    }

    /**
     * Updates drivebase with rotation override (for auto-centering)
     */
    public void updateDriveWithRotationOverride(double y, double x, double rotationCorrection) {
        // Apply cubic inputs for smooth control (ONLY to translation)
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);

        // Apply slow mode scaling ONLY to translation
        double maxPower = slowModeEnabled ? 0.25 : 1.0;

        // Field-centric rotation
        double heading = getHeading();
        double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

        // Use rotation correction directly - already computed by PID controller
        // Do NOT apply cubic or slow mode to rotation correction
        follower.setTeleOpDrive(rotatedY * maxPower, rotatedX * maxPower, rotationCorrection, false);
    }

    /**
     * Toggles slow mode
     */
    public void toggleSlowMode() {
        slowModeEnabled = !slowModeEnabled;
    }

    /**
     * Resets heading offset to current IMU reading
     */
    public void resetHeading() {
        imuOffset = imu.getAngularOrientation().firstAngle;
    }

    /**
     * Gets current robot heading with offset applied
     */
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle - imuOffset;
    }

    // Getters
    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

    public double getMaxPower() {
        return slowModeEnabled ? 0.25 : 1.0;
    }
}