package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueAutoFarConstants;

public class UpdateShootersFar {

    private DcMotorEx rightShooter, middleShooter, leftShooter;

    private double right_integral = 0, right_lastError = 0;
    private double middle_integral = 0, middle_lastError = 0;
    private double left_integral = 0, left_lastError = 0;

    private boolean shootersStarting = false;
    private double shooterStartTime = 0;

    private boolean rightShooterActive = false;
    private boolean middleShooterActive = false;
    private boolean leftShooterActive = false;

    public boolean shootersAtSpeed = false;
    public boolean rightShooterReady = false;
    public boolean middleShooterReady = false;
    public boolean leftShooterReady = false;

    public UpdateShootersFar(DcMotorEx right, DcMotorEx middle, DcMotorEx left) {
        this.rightShooter = right;
        this.middleShooter = middle;
        this.leftShooter = left;
    }

    public void startShooters(double currentTime) {
        shootersStarting = true;
        shooterStartTime = currentTime;
        rightShooterActive = true;
        middleShooterActive = false;
        leftShooterActive = true;
    }

    public void startLeftShooterOnly(double currentTime) {
        shootersStarting = true;
        shooterStartTime = currentTime;
        leftShooterActive = true;
        rightShooterActive = false;
        middleShooterActive = false;
    }

    public void stopLeftShooter() {
        leftShooter.setPower(0);
        left_integral = 0;
        left_lastError = 0;
        leftShooterReady = false;
        leftShooterActive = false;
    }

    public void startRightShooterOnly(double currentTime) {
        shootersStarting = true;
        shooterStartTime = currentTime;
        rightShooterActive = true;
        leftShooterActive = false;
        middleShooterActive = false;
    }

    public void stopRightShooter() {
        rightShooter.setPower(0);
        right_integral = 0;
        right_lastError = 0;
        rightShooterReady = false;
        rightShooterActive = false;
    }

    public void stopShooters() {
        rightShooter.setPower(0);
        middleShooter.setPower(0);
        leftShooter.setPower(0);

        shootersStarting = false;
        shootersAtSpeed = false;
        rightShooterActive = false;
        middleShooterActive = false;
        leftShooterActive = false;

        right_integral = middle_integral = left_integral = 0;
        right_lastError = middle_lastError = left_lastError = 0;
    }

    public void updateShooters(double dt, double currentTime) {
        if (shootersStarting) {
            double elapsedStartTime = currentTime - shooterStartTime;
            if (leftShooterActive && elapsedStartTime >= 0) {
                runShooterPID(leftShooter, dt, 2);
            }
            if (middleShooterActive && elapsedStartTime >= BlueAutoFarConstants.SHOOTER_START_DELAY) {
                runShooterPID(middleShooter, dt, 1);
            }
            if (rightShooterActive && elapsedStartTime >= BlueAutoFarConstants.SHOOTER_START_DELAY * 2) {
                runShooterPID(rightShooter, dt, 0);
            }

            if (elapsedStartTime >= BlueAutoFarConstants.SHOOTER_START_DELAY * 2) {
                shootersStarting = false;
            }
        } else {
            if (leftShooterActive) runShooterPID(leftShooter, dt, 2);
            if (middleShooterActive) runShooterPID(middleShooter, dt, 1);
            if (rightShooterActive) runShooterPID(rightShooter, dt, 0);
        }

        double rightRPM = getRPM(rightShooter);
        double middleRPM = getRPM(middleShooter);
        double leftRPM = getRPM(leftShooter);

        rightShooterReady = rightShooterActive && rightRPM >= BlueAutoFarConstants.RPM_THRESHOLD;
        middleShooterReady = middleShooterActive && middleRPM >= BlueAutoFarConstants.RPM_THRESHOLD;
        leftShooterReady = leftShooterActive && leftRPM >= BlueAutoFarConstants.RPM_THRESHOLD;

        shootersAtSpeed = (!rightShooterActive || rightShooterReady) &&
                (!middleShooterActive || middleShooterReady) &&
                (!leftShooterActive || leftShooterReady);
    }

    private void runShooterPID(DcMotorEx shooter, double dt, int index) {
        double rpm = getRPM(shooter);
        double error = BlueAutoFarConstants.TARGET_RPM - rpm;

        double derivative;
        double integral;

        if (index == 0) {
            right_integral += error * dt;
            right_integral = Range.clip(right_integral, -100, 100);
            derivative = (error - right_lastError) / dt;
            right_lastError = error;
            integral = right_integral;
        } else if (index == 1) {
            middle_integral += error * dt;
            middle_integral = Range.clip(middle_integral, -100, 100);
            derivative = (error - middle_lastError) / dt;
            middle_lastError = error;
            integral = middle_integral;
        } else {
            left_integral += error * dt;
            left_integral = Range.clip(left_integral, -100, 100);
            derivative = (error - left_lastError) / dt;
            left_lastError = error;
            integral = left_integral;
        }

        double P = BlueAutoFarConstants.SHOOTER_KP * error;
        double I = BlueAutoFarConstants.SHOOTER_KI * integral;
        double D = BlueAutoFarConstants.SHOOTER_KD * derivative;
        double F = BlueAutoFarConstants.SHOOTER_KF * BlueAutoFarConstants.TARGET_RPM;

        double power = Range.clip(P + I + D + F, 0.0, 1.0);
        shooter.setPower(power);
    }

    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;
    }
}
