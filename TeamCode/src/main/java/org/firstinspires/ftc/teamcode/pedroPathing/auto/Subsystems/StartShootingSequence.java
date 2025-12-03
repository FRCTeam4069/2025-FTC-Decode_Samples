package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;

public class StartShootingSequence {

    private UpdateShooters shooterSubsystem;
    private DcMotor intake;
    private Servo leftServo, middleServo, rightServo, leftPushServo, rightPushServo;

    private boolean waiting = false;
    private boolean blockersLowered = false;
    private double waitStartTime = 0;
    private double blockerServoDownTime = 0;

    public boolean isWaiting() {
        return waiting;
    }

    public StartShootingSequence(
            UpdateShooters shooterSubsystem,
            DcMotor intake,
            Servo leftServo, Servo middleServo, Servo rightServo,
            Servo leftPushServo, Servo rightPushServo) {
        this.shooterSubsystem = shooterSubsystem;
        this.intake = intake;
        this.leftServo = leftServo;
        this.middleServo = middleServo;
        this.rightServo = rightServo;
        this.leftPushServo = leftPushServo;
        this.rightPushServo = rightPushServo;
    }

    public void start(double currentTime) {
        waitStartTime = currentTime;
        blockerServoDownTime = 0;
        waiting = true;
        blockersLowered = false;

        shooterSubsystem.startShooters(currentTime);
        intake.setPower(0);

        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
    }

    public boolean update(double currentTime) {
        if (!waiting) return false;

        double elapsed = currentTime - waitStartTime;
        boolean timedOut = elapsed >= BlueFourCyclesConstants.SHOOTER_TIMEOUT;

        if ((shooterSubsystem.shootersAtSpeed || timedOut) && !blockersLowered) {
            leftServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
            middleServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
            rightServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
            blockerServoDownTime = currentTime;
            blockersLowered = true;
        }

        if (blockersLowered) {
            if (currentTime - blockerServoDownTime >= BlueFourCyclesConstants.PUSH_SERVO_DELAY) {
                leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
                rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
            }
        }

        if (elapsed >= 3.0) {
            stop();
            return true;
        }

        return false;
    }

    public void stop() {
        waiting = false;
        blockersLowered = false;

        shooterSubsystem.stopShooters();
        intake.setPower(0);

        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
    }
}
