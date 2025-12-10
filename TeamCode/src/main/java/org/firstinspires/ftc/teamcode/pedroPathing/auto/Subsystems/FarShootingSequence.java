package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueAutoFarConstants;


public class FarShootingSequence {

    private UpdateShootersFar shooterSubsystem;
    private Servo leftServo, middleServo, rightServo, leftPushServo, rightPushServo;
    private CRServo sorterServo, sorterServo2;
    private RevColorSensorV3 leftColor;

    private boolean waiting = false;
    private double startTime = 0;

    // Phase tracking
    private enum ShootingPhase {
        IDLE,
        LEFT_SHOOTER_SPINUP,
        LEFT_BALL_LOWER_BLOCKER,
        LEFT_BALL_FIRE_PUSH,
        LEFT_BALL_SHOT,
        RIGHT_SHOOTER_SPINUP,
        RIGHT_BALL_LOWER_BLOCKER,
        RIGHT_BALL_FIRE_PUSH,
        RIGHT_BALL_SHOT,
        TRANSFER_MIDDLE_BALL,
        MIDDLE_BALL_DETECTED,
        MIDDLE_SHOOTER_SPINUP,
        MIDDLE_BALL_LOWER_BLOCKER,
        MIDDLE_BALL_FIRE_PUSH,
        COMPLETE
    }

    private ShootingPhase currentPhase = ShootingPhase.IDLE;
    private double phaseStartTime = 0;

    public boolean isWaiting() {
        return waiting;
    }

    public FarShootingSequence(
            UpdateShootersFar shooterSubsystem,
            Servo leftServo, Servo middleServo, Servo rightServo,
            Servo leftPushServo, Servo rightPushServo,
            CRServo sorterServo, CRServo sorterServo2,
            RevColorSensorV3 leftColor) {
        this.shooterSubsystem = shooterSubsystem;
        this.leftServo = leftServo;
        this.middleServo = middleServo;
        this.rightServo = rightServo;
        this.leftPushServo = leftPushServo;
        this.rightPushServo = rightPushServo;
        this.sorterServo = sorterServo;
        this.sorterServo2 = sorterServo2;
        this.leftColor = leftColor;
    }

    public void start(double currentTime) {
        startTime = currentTime;
        phaseStartTime = currentTime;
        waiting = true;
        currentPhase = ShootingPhase.LEFT_SHOOTER_SPINUP;

        // Ensure all blockers and push servos are up
        // Remember: middleServo is actually left blocker, leftServo is middle blocker
        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);  // Middle blocker
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX); // Left blocker
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);

        // Stop sorter
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        // Start only left shooter
        shooterSubsystem.startLeftShooterOnly(currentTime);
    }

    public boolean update(double currentTime) {
        if (!waiting) return false;

        double phaseElapsed = currentTime - phaseStartTime;

        switch (currentPhase) {
            case LEFT_SHOOTER_SPINUP:
                // Wait for left shooter to reach speed
                if (shooterSubsystem.leftShooterReady || phaseElapsed >= 2.0) {
                    currentPhase = ShootingPhase.LEFT_BALL_LOWER_BLOCKER;
                    phaseStartTime = currentTime;
                    // Lower left blocker (middleServo is actually left blocker)
                    middleServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case LEFT_BALL_LOWER_BLOCKER:
                // Wait for blocker delay
                if (phaseElapsed >= BlueAutoFarConstants.PUSH_SERVO_DELAY) {
                    currentPhase = ShootingPhase.LEFT_BALL_FIRE_PUSH;
                    phaseStartTime = currentTime;
                    // Fire left push servo
                    leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case LEFT_BALL_FIRE_PUSH:
                // Wait for ball to shoot
                if (phaseElapsed >= 0.5) {
                    currentPhase = ShootingPhase.LEFT_BALL_SHOT;
                    phaseStartTime = currentTime;
                    // Stop left shooter
                    shooterSubsystem.stopLeftShooter();
                    // Reset left servos
                    middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
                    leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
                    // Start right shooter
                    shooterSubsystem.startRightShooterOnly(currentTime);
                }
                break;

            case LEFT_BALL_SHOT:
                // Wait for right shooter to spin up
                if (shooterSubsystem.rightShooterReady || phaseElapsed >= 2.0) {
                    currentPhase = ShootingPhase.RIGHT_BALL_LOWER_BLOCKER;
                    phaseStartTime = currentTime;
                    // Lower right blocker
                    rightServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case RIGHT_BALL_LOWER_BLOCKER:
                // Wait for blocker delay
                if (phaseElapsed >= BlueAutoFarConstants.PUSH_SERVO_DELAY) {
                    currentPhase = ShootingPhase.RIGHT_BALL_FIRE_PUSH;
                    phaseStartTime = currentTime;
                    // Fire right push servo
                    rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case RIGHT_BALL_FIRE_PUSH:
                // Wait for ball to shoot
                if (phaseElapsed >= 0.5) {
                    currentPhase = ShootingPhase.RIGHT_BALL_SHOT;
                    phaseStartTime = currentTime;
                    // Stop right shooter
                    shooterSubsystem.stopRightShooter();
                    // Reset right servos
                    rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
                    rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
                    // Start transferring middle ball to left
                    sorterServo.setPower(-1.0);
                    sorterServo2.setPower(-1.0);
                }
                break;

            case RIGHT_BALL_SHOT:
                // Wait for middle ball to reach left sensor
                int l = leftColor.green();
                if (l > 100) {
                    currentPhase = ShootingPhase.MIDDLE_BALL_DETECTED;
                    phaseStartTime = currentTime;
                    // Stop sorter
                    sorterServo.setPower(-1.0);
                    sorterServo2.setPower(-1.0);
                    // Start left shooter again
                    shooterSubsystem.startLeftShooterOnly(currentTime);
                }
                // Safety timeout
                if (phaseElapsed >= 5.0) {
                    currentPhase = ShootingPhase.MIDDLE_BALL_DETECTED;
                    phaseStartTime = currentTime;
                    sorterServo.setPower(0);
                    sorterServo2.setPower(0);
                    shooterSubsystem.startLeftShooterOnly(currentTime);
                }
                break;

            case MIDDLE_BALL_DETECTED:
                // Wait for left shooter to spin up
                if (shooterSubsystem.leftShooterReady || phaseElapsed >= 2.0) {
                    currentPhase = ShootingPhase.MIDDLE_BALL_LOWER_BLOCKER;
                    phaseStartTime = currentTime;
                    // Lower left blocker (middleServo is actually left blocker)
                    middleServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case MIDDLE_BALL_LOWER_BLOCKER:
                // Wait for blocker delay
                if (phaseElapsed >= BlueAutoFarConstants.PUSH_SERVO_DELAY) {
                    currentPhase = ShootingPhase.MIDDLE_BALL_FIRE_PUSH;
                    phaseStartTime = currentTime;
                    // Fire left push servo
                    leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                }
                break;

            case MIDDLE_BALL_FIRE_PUSH:
                // Wait for ball to shoot
                if (phaseElapsed >= 0.5) {
                    currentPhase = ShootingPhase.COMPLETE;
                    stop();
                    return true;
                }
                break;
        }

        // Safety timeout
        if (currentTime - startTime >= 15.0) {
            stop();
            return true;
        }

        return false;
    }

    public void stop() {
        waiting = false;
        currentPhase = ShootingPhase.IDLE;

        // Stop sorter
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        // Stop all shooters
        shooterSubsystem.stopShooters();

        // Reset all servos
        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
    }

    public void cancel() {
        // Emergency cancel for timeout - immediately stop everything
        stop();
    }
}