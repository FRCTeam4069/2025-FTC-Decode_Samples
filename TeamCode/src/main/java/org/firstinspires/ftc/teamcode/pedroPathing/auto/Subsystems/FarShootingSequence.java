package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueAutoFarConstants;

public class FarShootingSequence {

    private UpdateShootersFar shooterSubsystem;
    private Servo leftServo, middleServo, rightServo, leftPushServo, rightPushServo;
    private CRServo sorterServo, sorterServo2;
    private RevColorSensorV3 leftColor;

    private Follower follower;
    private BNO055IMU imu;
    private Limelight3A limelight;
    private double imuOffset = 0.0;

    // SAFE FLAGS — these are ALWAYS valid, even before start() is called
    private boolean waiting = false;
    private boolean centering = false;
    private boolean centered = false;

    // Internal state (your original variables)
    private boolean sideBallsBlockersLowered = false;
    private boolean sideBallsPushServosFired = false;
    private boolean sideBallsShot = false;
    private double waitStartTime = 0;
    private double sideBallsBlockerDownTime = 0;
    private double sideBallsPushServoFireTime = 0;
    private double sideBallsShotTime = 0;

    private boolean shootersReachedSpeed = false;
    private boolean shootersPreSpun = false;

    private boolean centeringStarted = false;
    private boolean centeringComplete = false;
    private double centeringStartTime = 0;
    private double lastCenteringError = 0.0;
    private double centeringSettleStartTime = 0;
    private boolean centeringSettled = false;

    private boolean leftPushServoRaised = false;
    private boolean transferringMiddleBall = false;
    private boolean middleBallDetected = false;
    private boolean middleBallBlockerLowered = false;
    private boolean middleBallPushServoFired = false;
    private double transferStartTime = 0;
    private double middleBallBlockerDownTime = 0;
    private double middleBallPushServoFireTime = 0;

    // Centering constants
    private static final double CENTERING_KP = 0.0075;
    private static final double CENTERING_KD = 0.0005;
    private static final double CENTERING_ANGLE_TOLERANCE = 2.0;
    private static final double CENTERING_MAX_POWER = 0.4;
    private static final double CENTERING_TIMEOUT = 2.0;
    private static final double CENTERING_SETTLE_TIME = 0.15;
    public static double CENTERING_OFFSET_DEGREES = 40; // tweak as needed

    // Public safe getters
    public boolean isWaiting()  { return waiting; }
    public boolean isCentering() { return centering; }
    public boolean isCentered()  { return centered; }

    public FarShootingSequence(
            UpdateShootersFar shooterSubsystem,
            Servo leftServo, Servo middleServo, Servo rightServo,
            Servo leftPushServo, Servo rightPushServo,
            CRServo sorterServo, CRServo sorterServo2,
            RevColorSensorV3 leftColor,
            Follower follower, BNO055IMU imu, Limelight3A limelight) {

        this.shooterSubsystem = shooterSubsystem;
        this.leftServo = leftServo;
        this.middleServo = middleServo;
        this.rightServo = rightServo;
        this.leftPushServo = leftPushServo;
        this.rightPushServo = rightPushServo;
        this.sorterServo = sorterServo;
        this.sorterServo2 = sorterServo2;
        this.leftColor = leftColor;
        this.follower = follower;
        this.imu = imu;
        this.limelight = limelight;

        try {
            imuOffset = imu.getAngularOrientation().firstAngle;
        } catch (Exception e) {
            imuOffset = 0.0;
        }

        // These are safe even before start() is ever called
        waiting = false;
        centering = false;
        centered = false;
    }

    public void start(double currentTime) {
        startInternal(currentTime, false);
    }

    public void startWithPreSpunShooters(double currentTime) {
        startInternal(currentTime, true);
    }

    private void startInternal(double currentTime, boolean preSpun) {
        waitStartTime = currentTime;

        waiting = true;
        centering = false;
        centered = false;

        // Reset all internal state
        sideBallsBlockersLowered = false;
        sideBallsPushServosFired = false;
        sideBallsShot = false;
        shootersReachedSpeed = false;
        shootersPreSpun = preSpun;
        leftPushServoRaised = false;
        transferringMiddleBall = false;
        middleBallDetected = false;
        middleBallBlockerLowered = false;
        middleBallPushServoFired = false;

        centeringStarted = false;
        centeringComplete = false;
        centeringSettled = false;

        if (!preSpun) {
            shooterSubsystem.startSideShoostersOnly(currentTime);
        }

        // Servos up
        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);

        sorterServo.setPower(0);
        sorterServo2.setPower(0);
    }

    public boolean update(double currentTime) {
        if (!waiting) return false;

        double elapsed = currentTime - waitStartTime;

        // PHASE 0 – Limelight Centering
        if (!centeringStarted && !shootersReachedSpeed) {
            centeringStarted = true;
            centeringStartTime = currentTime;
            centering = true;
            centered = false;
        }

        if (centeringStarted && !centeringComplete) {
            double centeringElapsed = currentTime - centeringStartTime;

            double imuYawDegrees = getHeading();
            try { limelight.updateRobotOrientation(imuYawDegrees); } catch (Exception ignored) {}

            LLResult result = limelight.getLatestResult();

            // Timeout
            if (centeringElapsed >= CENTERING_TIMEOUT) {
                follower.setTeleOpDrive(0, 0, 0);
                centeringComplete = true;
                centering = false;
                centered = false;
                // continue to shooting
            }
            // No target
            else if (result == null || !result.isValid()) {
                if (centeringElapsed >= 0.5) {
                    follower.setTeleOpDrive(0, 0, 0);
                    centeringComplete = true;
                    centering = false;
                    centered = false;
                }
            }
            // Target visible → PD control
            else {
                double tx = result.getTx();
                double errorAngle = tx - CENTERING_OFFSET_DEGREES;

                double dt = 0.02;
                double deriv = (errorAngle - lastCenteringError) / dt;
                lastCenteringError = errorAngle;

                double correction = -(CENTERING_KP * errorAngle + CENTERING_KD * deriv);
                correction = clamp(correction, -CENTERING_MAX_POWER, CENTERING_MAX_POWER);

                if (Math.abs(errorAngle) < CENTERING_ANGLE_TOLERANCE) {
                    if (!centeringSettled) {
                        centeringSettled = true;
                        centeringSettleStartTime = currentTime;
                    }
                    if (currentTime - centeringSettleStartTime >= CENTERING_SETTLE_TIME) {
                        follower.setTeleOpDrive(0, 0, 0);
                        centeringComplete = true;
                        centering = false;
                        centered = true;
                    } else {
                        follower.setTeleOpDrive(0, 0, correction * 0.5);
                    }
                } else {
                    centeringSettled = false;
                    follower.setTeleOpDrive(0, 0, correction);
                }
            }
            return false; // still centering
        }

        // PHASE 1 – Wait for shooters
        if (centeringComplete && (shooterSubsystem.shootersAtSpeed || elapsed >= BlueAutoFarConstants.SHOOTER_TIMEOUT || shootersPreSpun) && !shootersReachedSpeed) {
            shootersReachedSpeed = true;
        }

        // PHASE 2 – Lower side blockers
        if (shootersReachedSpeed && !sideBallsBlockersLowered) {
            middleServo.setPosition(BlueAutoFarConstants.SERVO_MIN); // left blocker
            rightServo.setPosition(BlueAutoFarConstants.SERVO_MIN);  // right blocker
            sideBallsBlockerDownTime = currentTime;
            sideBallsBlockersLowered = true;
        }

        // PHASE 3 – Fire side push servos
        if (sideBallsBlockersLowered && !sideBallsPushServosFired && (currentTime - sideBallsBlockerDownTime >= BlueAutoFarConstants.PUSH_SERVO_DELAY)) {
            rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
            leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
            sideBallsPushServoFireTime = currentTime;
            sideBallsPushServosFired = true;
        }

        // PHASE 4 – Side balls shot
        if (sideBallsPushServosFired && !sideBallsShot && (currentTime - sideBallsPushServoFireTime >= 0.5)) {
            sideBallsShot = true;
            sideBallsShotTime = currentTime;
            shooterSubsystem.stopRightShooter();

            middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
            rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
            rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
            leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        }

        // PHASE 5 – Start middle ball transfer
        if (sideBallsShot && !leftPushServoRaised && (currentTime - sideBallsShotTime >= 0.2)) {
            leftPushServoRaised = true;
            transferringMiddleBall = true;
            transferStartTime = currentTime;
            sorterServo.setPower(-1.0);
            sorterServo2.setPower(-1.0);
        }

        // PHASE 6 – Detect middle ball
        if (transferringMiddleBall && !middleBallDetected) {
            if (leftColor.green() > 200 || (currentTime - transferStartTime >= 5.0)) {
                middleBallDetected = true;
                middleBallBlockerDownTime = currentTime;
            }
        }

        // PHASE 7 – Lower middle blocker
        if (middleBallDetected && !middleBallBlockerLowered) {
            if (shooterSubsystem.leftShooterReady || (currentTime - middleBallBlockerDownTime >= 1.0)) {
                middleServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
                middleBallBlockerLowered = true;
            }
        }

        // PHASE 8 – Fire middle push servo
        if (middleBallBlockerLowered && !middleBallPushServoFired && (currentTime - middleBallBlockerDownTime >= BlueAutoFarConstants.PUSH_SERVO_DELAY)) {
            leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MIN);
            middleBallPushServoFireTime = currentTime;
            middleBallPushServoFired = true;
        }

        // Finish
        if (middleBallPushServoFired && (currentTime - middleBallPushServoFireTime >= 0.5)) {
            sorterServo.setPower(0);
            sorterServo2.setPower(0);
            stop();
            return true;
        }

        // Safety timeout
        if (elapsed >= BlueAutoFarConstants.SHOOTER_TIMEOUT + 5.0) {
            stop();
            return true;
        }

        return false;
    }

    public void stop() {
        waiting = false;
        centering = false;
        centered = false;

        follower.setTeleOpDrive(0, 0, 0);
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);

        // Reset internal flags so next shot works perfectly
        sideBallsBlockersLowered = false;
        sideBallsPushServosFired = false;
        sideBallsShot = false;
        shootersReachedSpeed = false;
        shootersPreSpun = false;
        leftPushServoRaised = false;
        transferringMiddleBall = false;
        middleBallDetected = false;
        middleBallBlockerLowered = false;
        middleBallPushServoFired = false;
        centeringStarted = false;
        centeringComplete = false;
        centeringSettled = false;
    }

    public void cancel() {
        waiting = false;
        centering = false;
        centered = false;
        centeringStarted = false;
        centeringComplete = false;

        follower.setTeleOpDrive(0, 0, 0);
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double getHeading() {
        try {
            return imu.getAngularOrientation().firstAngle - imuOffset;
        } catch (Exception e) {
            return -imuOffset;
        }
    }
}