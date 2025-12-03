package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;

public class UpdateShooters {

    private DcMotorEx rightShooter, middleShooter, leftShooter;

    private double right_integral = 0, right_lastError = 0;
    private double middle_integral = 0, middle_lastError = 0;
    private double left_integral = 0, left_lastError = 0;

    private boolean shootersStarting = false;
    private double shooterStartTime = 0;

    public boolean shootersAtSpeed = false;
    public boolean rightShooterReady = false;
    public boolean middleShooterReady = false;
    public boolean leftShooterReady = false;

    public UpdateShooters(DcMotorEx right, DcMotorEx middle, DcMotorEx left) {
        this.rightShooter = right;
        this.middleShooter = middle;
        this.leftShooter = left;
    }

    public void startShooters(double currentTime) {
        shootersStarting = true;
        shooterStartTime = currentTime;
    }

    public void stopShooters() {
        rightShooter.setPower(0);
        middleShooter.setPower(0);
        leftShooter.setPower(0);

        shootersStarting = false;
        right_integral = middle_integral = left_integral = 0;
    }

    public void updateShooters(double dt, double currentTime) {

        if (shootersStarting) {
            double elapsedStartTime = currentTime - shooterStartTime;

            if (elapsedStartTime >= 0) {
                runShooterPID(rightShooter, dt, 0);
            }

            if (elapsedStartTime >= BlueFourCyclesConstants.SHOOTER_START_DELAY) {
                runShooterPID(middleShooter, dt, 1);
            }

            if (elapsedStartTime >= BlueFourCyclesConstants.SHOOTER_START_DELAY * 2) {
                runShooterPID(leftShooter, dt, 2);
                shootersStarting = false;
            }

            return;
        }

        double rightRPM = getRPM(rightShooter);
        double middleRPM = getRPM(middleShooter);
        double leftRPM = getRPM(leftShooter);

        rightShooterReady  = rightRPM  >= BlueFourCyclesConstants.RPM_THRESHOLD;
        middleShooterReady = middleRPM >= BlueFourCyclesConstants.RPM_THRESHOLD;
        leftShooterReady   = leftRPM   >= BlueFourCyclesConstants.RPM_THRESHOLD;

        shootersAtSpeed = rightShooterReady && middleShooterReady && leftShooterReady;

        runShooterPID(rightShooter, dt, 0);
        runShooterPID(middleShooter, dt, 1);
        runShooterPID(leftShooter, dt, 2);
    }

    private void runShooterPID(DcMotorEx shooter, double dt, int index) {
        double rpm = getRPM(shooter);
        double error = BlueFourCyclesConstants.TARGET_RPM - rpm;

        double derivative;
        double integral;

        if (index == 0) {
            right_integral += error * dt;
            derivative = (error - right_lastError) / dt;
            right_lastError = error;
            integral = right_integral;
        } else if (index == 1) {
            middle_integral += error * dt;
            derivative = (error - middle_lastError) / dt;
            middle_lastError = error;
            integral = middle_integral;
        } else {
            left_integral += error * dt;
            derivative = (error - left_lastError) / dt;
            left_lastError = error;
            integral = left_integral;
        }

        double P = BlueFourCyclesConstants.SHOOTER_KP * error;
        double I = BlueFourCyclesConstants.SHOOTER_KI * integral;
        double D = BlueFourCyclesConstants.SHOOTER_KD * derivative;
        double F = BlueFourCyclesConstants.SHOOTER_KF * BlueFourCyclesConstants.TARGET_RPM;

        double power = Range.clip(P + I + D + F, 0.0, 1.0);
        shooter.setPower(power);
    }

    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;
    }
}
