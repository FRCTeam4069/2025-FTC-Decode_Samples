package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;

public class StartShootingSequence {

    private UpdateShooters shooterSubsystem;
    private Servo leftServo, middleServo, rightServo, leftPushServo, rightPushServo;
    private CRServo sorterServo, sorterServo2;
    private RevColorSensorV3 leftColor;

    // Limelight centering
    private Follower follower;
    private BNO055IMU imu;
    private Limelight3A limelight;
    private double imuOffset = 0.0;

    private boolean waiting = false;
    private boolean sideBallsBlockersLowered = false;
    private boolean sideBallsPushServosFired = false;
    private boolean sideBallsShot = false;
    private double waitStartTime = 0;
    private double sideBallsBlockerDownTime = 0;
    private double sideBallsPushServoFireTime = 0;
    private double sideBallsShotTime = 0;

    // Track if shooters have reached speed
    private boolean shootersReachedSpeed = false;
    // Track if shooters were pre-spun (started before this sequence)
    private boolean shootersPreSpun = false;

    // Limelight centering phase
    private boolean centeringStarted = false;
    private boolean centeringComplete = false;
    private double centeringStartTime = 0;
    private double lastCenteringError = 0.0;

    // Centering tuning
    private static final double CENTERING_KP = 0.0075;
    private static final double CENTERING_KD = 0.0005;
    private static final double CENTERING_ANGLE_TOLERANCE = 2.0; // degrees
    private static final double CENTERING_MAX_POWER = 0.4;
    private static final double CENTERING_TIMEOUT = 2.0; // seconds
    private static final double CENTERING_SETTLE_TIME = 0.15; // time to stay centered before proceeding
    private double centeringSettleStartTime = 0;
    private boolean centeringSettled = false;

    // Track middle ball transfer sequence
    private boolean leftPushServoRaised = false;
    private boolean transferringMiddleBall = false;
    private boolean middleBallDetected = false;
    private boolean middleBallBlockerLowered = false;
    private boolean middleBallPushServoFired = false;
    private double transferStartTime = 0;
    private double middleBallBlockerDownTime = 0;
    private double middleBallPushServoFireTime = 0;

    public boolean isWaiting() {
        return waiting;
    }

    public StartShootingSequence(
            UpdateShooters shooterSubsystem,
            Servo leftServo, Servo middleServo, Servo rightServo,
            Servo leftPushServo, Servo rightPushServo,
            CRServo sorterServo, CRServo sorterServo2,
            RevColorSensorV3 leftColor,
            Follower follower,
            BNO055IMU imu,
            Limelight3A limelight) {
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

        // Initialize IMU offset
        try {
            imuOffset = imu.getAngularOrientation().firstAngle;
        } catch (Exception e) {
            imuOffset = 0.0;
        }
    }

    public void start(double currentTime) {
        startInternal(currentTime, false);
    }

    // New method for when shooters are already spinning
    public void startWithPreSpunShooters(double currentTime) {
        startInternal(currentTime, true);
    }

    private void startInternal(double currentTime, boolean preSpun) {
        waitStartTime = currentTime;
        sideBallsBlockerDownTime = 0;
        sideBallsPushServoFireTime = 0;
        sideBallsShotTime = 0;
        waiting = true;
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
        transferStartTime = 0;
        middleBallBlockerDownTime = 0;
        middleBallPushServoFireTime = 0;

        // Reset centering state
        centeringStarted = false;
        centeringComplete = false;
        centeringStartTime = 0;
        lastCenteringError = 0.0;
        centeringSettled = false;
        centeringSettleStartTime = 0;

        // Only start shooters if they weren't pre-spun
        if (!preSpun) {
            shooterSubsystem.startSideShoostersOnly(currentTime);
        }

        // Ensure blocker servos are up (remember: leftServo is actually middle, middleServo is actually left)
        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);  // Actually middle blocker
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX); // Actually left blocker
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);  // Actually middle push
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);

        // Stop sorter initially
        sorterServo.setPower(0);
        sorterServo2.setPower(0);
    }

    public boolean update(double currentTime) {
        if (!waiting) return false;

        double elapsed = currentTime - waitStartTime;
        boolean timedOut = elapsed >= BlueFourCyclesConstants.SHOOTER_TIMEOUT;

        // === PHASE 0: Limelight centering (new phase) ===
        if (!centeringStarted && !shootersReachedSpeed) {
            centeringStarted = true;
            centeringStartTime = currentTime;
        }

        if (centeringStarted && !centeringComplete) {
            double centeringElapsed = currentTime - centeringStartTime;

            // Update IMU orientation for limelight
            double imuYawDegrees = getHeading();
            try {
                limelight.updateRobotOrientation(imuYawDegrees);
            } catch (Exception ignored) {}

            LLResult result = limelight.getLatestResult();

            // Check for centering timeout
            if (centeringElapsed >= CENTERING_TIMEOUT) {
                // Timeout - stop centering and continue
                follower.setTeleOpDrive(0, 0, 0);
                centeringComplete = true;
            }
            // Check if we have a valid target
            else if (result == null || !result.isValid()) {
                // No target visible - timeout after 0.5 seconds
                if (centeringElapsed >= 0.5) {
                    follower.setTeleOpDrive(0, 0, 0);
                    centeringComplete = true;
                }
            } else {
                // Target visible - perform centering
                double tx = result.getTx();
                double errorAngle = tx;

                // Calculate derivative
                double dt = 0.02; // Approximate loop time
                double derivAngle = (errorAngle - lastCenteringError) / dt;
                lastCenteringError = errorAngle;

                // PD control for rotation
                double correctionRotation = -(CENTERING_KP * errorAngle + CENTERING_KD * derivAngle);
                correctionRotation = clamp(correctionRotation, -CENTERING_MAX_POWER, CENTERING_MAX_POWER);

                // Check if centered within tolerance
                boolean isCentered = Math.abs(errorAngle) < CENTERING_ANGLE_TOLERANCE;

                if (isCentered) {
                    // Start settle timer if just reached center
                    if (!centeringSettled) {
                        centeringSettled = true;
                        centeringSettleStartTime = currentTime;
                    }

                    // Check if we've been centered long enough
                    if (currentTime - centeringSettleStartTime >= CENTERING_SETTLE_TIME) {
                        // Successfully centered and settled
                        follower.setTeleOpDrive(0, 0, 0);
                        centeringComplete = true;
                    } else {
                        // Still settling - apply small correction
                        follower.setTeleOpDrive(0, 0, correctionRotation * 0.5);
                    }
                } else {
                    // Not centered - reset settle state and continue centering
                    centeringSettled = false;
                    follower.setTeleOpDrive(0, 0, correctionRotation);
                }
            }

            // Don't proceed until centering is complete
            return false;
        }

        // === PHASE 1: Side shooters spin up (or skip if pre-spun) ===
        if (centeringComplete && ((shooterSubsystem.shootersAtSpeed || timedOut || shootersPreSpun) && !shootersReachedSpeed)) {
            shootersReachedSpeed = true;
        }

        // === PHASE 2: Lower BOTH side blockers and shoot BOTH side balls simultaneously ===
        if (shootersReachedSpeed && !sideBallsBlockersLowered) {
            // Lower BOTH right blocker and left blocker at the same time
            // Remember: middleServo is actually left blocker
            middleServo.setPosition(BlueFourCyclesConstants.SERVO_MIN); // Actually left blocker
            rightServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);   // Right blocker
            sideBallsBlockerDownTime = currentTime;
            sideBallsBlockersLowered = true;
        }

        // Fire BOTH push servos at the same time after blocker delay
        if (sideBallsBlockersLowered && !sideBallsPushServosFired) {
            if (currentTime - sideBallsBlockerDownTime >= BlueFourCyclesConstants.PUSH_SERVO_DELAY) {
                // Fire BOTH side push servos simultaneously
                // Remember: leftPushServo is actually middle push, so we skip it
                rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);  // Right push servo
                leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);      // Keep left blocker down
                sideBallsPushServoFireTime = currentTime;
                sideBallsPushServosFired = true;
            }
        }

        // === PHASE 3: Wait for side balls to shoot, then reset side servos ===
        if (sideBallsPushServosFired && !sideBallsShot) {
            if (currentTime - sideBallsPushServoFireTime >= 0.5) {
                // Both side balls have shot
                sideBallsShot = true;
                sideBallsShotTime = currentTime;

                // Stop right shooter (left shooter stays on)
                shooterSubsystem.stopRightShooter();

                // Reset side servos to get out of the way
                middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX); // Actually left blocker - raise it
                rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);   // Right blocker - raise it
                rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX); // Right push - reset it

                // IMPORTANT: Raise left push servo to get out of the way (leftPushServo is actually middle push)
                leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX); // Raise middle push servo
            }
        }

        // === PHASE 4: Raise left push servo completely before starting transfer ===
        if (sideBallsShot && !leftPushServoRaised) {
            if (currentTime - sideBallsShotTime >= 0.2) { // Small delay to ensure servo is up
                leftPushServoRaised = true;

                // NOW start transferring middle ball to left
                transferringMiddleBall = true;
                transferStartTime = currentTime;
                // Both sorter servos spin left at -1
                sorterServo.setPower(-1.0);
                sorterServo2.setPower(-1.0);
            }
        }

        // === PHASE 5: Wait for middle ball to reach left sensor ===
        if (transferringMiddleBall && !middleBallDetected) {
            int l = leftColor.green();
            if (l > 200) {
                // Ball detected on left sensor
                sorterServo.setPower(-1.0);
                sorterServo2.setPower(-1.0);
                middleBallDetected = true;
                middleBallBlockerDownTime = currentTime;
            }

            // Safety timeout for transfer
            if (currentTime - transferStartTime >= 3) {
                sorterServo.setPower(0);
                sorterServo2.setPower(0);
                middleBallDetected = true;
                middleBallBlockerDownTime = currentTime;
            }
        }

        // === PHASE 6: Wait for shooter to be at RPM, then lower left blocker for middle ball ===
        if (middleBallDetected && !middleBallBlockerLowered) {
            // Check if left shooter is at speed
            if (shooterSubsystem.leftShooterReady) {
                // Lower left blocker (middleServo is actually left blocker)
                middleServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
                middleBallBlockerLowered = true;
            }

            // Safety timeout - lower blocker anyway after 1 second
            if (currentTime - middleBallBlockerDownTime >= 1.0) {
                middleServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
                middleBallBlockerLowered = true;
            }
        }

        // === PHASE 7: Fire left push servo for middle ball ===
        if (middleBallBlockerLowered && !middleBallPushServoFired) {
            if (currentTime - middleBallBlockerDownTime >= BlueFourCyclesConstants.PUSH_SERVO_DELAY) {
                // Fire the actual left push servo
                // Since leftPushServo is wired as middle push, and middleServo is wired as left blocker,
                // we need to figure out the actual left push servo
                // Looking at the pattern: if middleServo = left blocker, and maybe we don't have a separate left push?
                // Let's just keep the blocker down and the ball should shoot
                middleServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);// Keep left blocker down
                leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MIN);
                middleBallPushServoFireTime = currentTime;
                middleBallPushServoFired = true;
            }
        }

        // === PHASE 8: Complete sequence after middle ball shoots ===
        if (middleBallPushServoFired) {
            if (currentTime - middleBallPushServoFireTime >= 0.5) {
                sorterServo.setPower(0.0);
                sorterServo2.setPower(0.0);
                stop();
                return true;
            }
        }

        // Safety timeout
        if (elapsed >= BlueFourCyclesConstants.SHOOTER_TIMEOUT + 5.0) {
            stop();
            return true;
        }

        return false;
    }

    public void stop() {
        waiting = false;
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

        // Stop robot movement
        follower.setTeleOpDrive(0, 0, 0);

        // Stop sorter
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        // Reset all servos
        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
    }

    public void cancel() {
        // Emergency cancel for timeout - immediately stop everything
        follower.setTeleOpDrive(0, 0, 0);

        // Force complete the sequence without finishing
        waiting = false;
        centeringStarted = false;
        centeringComplete = false;

        // Stop all motors and servos
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        // Reset all servos
        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
    }

    // Helper methods for limelight centering
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getHeading() {
        try {
            return imu.getAngularOrientation().firstAngle - imuOffset;
        } catch (Exception e) {
            return -imuOffset;
        }
    }

    // Public getter for centering status (for telemetry)
    public boolean isCentering() {
        return centeringStarted && !centeringComplete;
    }

    public boolean isCentered() {
        return centeringComplete;
    }
}