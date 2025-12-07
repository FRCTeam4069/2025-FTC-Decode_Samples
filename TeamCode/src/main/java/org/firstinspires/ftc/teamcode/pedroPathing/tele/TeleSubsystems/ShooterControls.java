package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Controls the three shooter motors with independent PIDF loops
 */
public class ShooterControls {

    private DcMotorEx rightShooter;
    private DcMotorEx middleShooter;
    private DcMotorEx leftShooter;

    // PID state for each shooter
    private double right_integral = 0.0;
    private double right_lastError = 0.0;

    private double middle_integral = 0.0;
    private double middle_lastError = 0.0;

    private double left_integral = 0.0;
    private double left_lastError = 0.0;

    // Enable flags
    private boolean rightShooterEnabled = false;
    private boolean middleShooterEnabled = false;
    private boolean leftShooterEnabled = false;

    public ShooterControls(DcMotorEx rightShooter, DcMotorEx middleShooter, DcMotorEx leftShooter) {
        this.rightShooter = rightShooter;
        this.middleShooter = middleShooter;
        this.leftShooter = leftShooter;

        // Configure motors
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        middleShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Updates all shooter motors with PIDF control
     */
    public void update(double dt, double rightTarget, double middleTarget, double leftTarget) {
        // Get current RPMs
        double rightRPM = getRightRPM();
        double middleRPM = getMiddleRPM();
        double leftRPM = getLeftRPM();

        // Calculate and apply powers
        double rightPower = 0.0;
        double middlePower = 0.0;
        double leftPower = 0.0;

        if (rightShooterEnabled) {
            rightPower = calculatePIDFPower(rightTarget, rightRPM, dt,
                    RobotConstants.right_kP, RobotConstants.right_kI, RobotConstants.right_kD, RobotConstants.right_kF,
                    0); // 0 = right shooter
        }

        if (middleShooterEnabled) {
            middlePower = calculatePIDFPower(middleTarget, middleRPM, dt,
                    RobotConstants.middle_kP, RobotConstants.middle_kI, RobotConstants.middle_kD, RobotConstants.middle_kF,
                    1); // 1 = middle shooter
        }

        if (leftShooterEnabled) {
            leftPower = calculatePIDFPower(leftTarget, leftRPM, dt,
                    RobotConstants.left_kP, RobotConstants.left_kI, RobotConstants.left_kD, RobotConstants.left_kF,
                    2); // 2 = left shooter
        }

        rightShooter.setPower(rightPower);
        middleShooter.setPower(middlePower);
        leftShooter.setPower(leftPower);
    }

    /**
     * Calculates PIDF power for a single shooter
     */
    private double calculatePIDFPower(double target, double current, double dt,
                                      double kP, double kI, double kD, double kF, int motorIndex) {
        double error = target - current;
        double integral, lastError, derivative;

        // Update appropriate state based on motor index
        switch (motorIndex) {
            case 0: // right
                right_integral += error * dt;
                derivative = (error - right_lastError) / dt;
                right_lastError = error;
                integral = right_integral;
                break;
            case 1: // middle
                middle_integral += error * dt;
                derivative = (error - middle_lastError) / dt;
                middle_lastError = error;
                integral = middle_integral;
                break;
            case 2: // left
                left_integral += error * dt;
                derivative = (error - left_lastError) / dt;
                left_lastError = error;
                integral = left_integral;
                break;
            default:
                return 0.0;
        }

        double p = kP * error;
        double i = kI * integral;
        double d = kD * derivative;
        double f = kF * target;
        double power = p + i + d + f;

        return Range.clip(power, 0.0, 1.0);
    }

    /**
     * Gets current RPM for right shooter
     */
    public double getRightRPM() {
        return rightShooter.getVelocity() * 60.0 / RobotConstants.TICKS_PER_REV;
    }

    /**
     * Gets current RPM for middle shooter
     */
    public double getMiddleRPM() {
        return middleShooter.getVelocity() * 60.0 / RobotConstants.TICKS_PER_REV;
    }

    /**
     * Gets current RPM for left shooter
     */
    public double getLeftRPM() {
        return leftShooter.getVelocity() * 60.0 / RobotConstants.TICKS_PER_REV;
    }

    /**
     * Checks if all enabled shooters are at target RPM
     */
    public boolean areAllEnabledShootersAtTarget(double rightTarget, double middleTarget, double leftTarget) {
        boolean anyEnabled = leftShooterEnabled || middleShooterEnabled || rightShooterEnabled;
        if (!anyEnabled) return false;

        boolean allAtTarget = true;

        if (leftShooterEnabled) {
            allAtTarget &= Math.abs(getLeftRPM() - leftTarget) <= RobotConstants.rpm_tolerance;
        }
        if (middleShooterEnabled) {
            allAtTarget &= Math.abs(getMiddleRPM() - middleTarget) <= RobotConstants.rpm_tolerance;
        }
        if (rightShooterEnabled) {
            allAtTarget &= Math.abs(getRightRPM() - rightTarget) <= RobotConstants.rpm_tolerance;
        }

        return allAtTarget;
    }

    /**
     * Enables all shooters
     */
    public void enableAllShooters() {
        rightShooterEnabled = true;
        middleShooterEnabled = true;
        leftShooterEnabled = true;
    }

    // Enable/disable methods
    public void setRightShooterEnabled(boolean enabled) {
        this.rightShooterEnabled = enabled;
    }

    public void setMiddleShooterEnabled(boolean enabled) {
        this.middleShooterEnabled = enabled;
    }

    public void setLeftShooterEnabled(boolean enabled) {
        this.leftShooterEnabled = enabled;
    }

    public void disableAllShooters() {
        rightShooterEnabled = false;
        middleShooterEnabled = false;
        leftShooterEnabled = false;
    }

    // Getters
    public boolean isRightShooterEnabled() {
        return rightShooterEnabled;
    }

    public boolean isMiddleShooterEnabled() {
        return middleShooterEnabled;
    }

    public boolean isLeftShooterEnabled() {
        return leftShooterEnabled;
    }

    public boolean isAnyShooterEnabled() {
        return leftShooterEnabled || middleShooterEnabled || rightShooterEnabled;
    }
}