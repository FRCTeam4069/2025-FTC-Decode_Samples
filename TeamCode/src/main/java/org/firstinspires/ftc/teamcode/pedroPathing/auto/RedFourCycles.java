package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Close Four Cycles", group = "Autonomous")
@Configurable
public class RedFourCycles extends OpMode {

    // -----------------------------------------------------------------------
    // Shooter PIDF constants (matching TeleOp)
    // -----------------------------------------------------------------------
    private static final double SHOOTER_KP = 0.0002;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.0001;
    private static final double SHOOTER_KF = 0.0004;
    private static final double TARGET_RPM = 3000.0;
    private static final double RPM_THRESHOLD = 2800.0; // Lower servos when within 100 RPM

    private static final double TICKS_PER_REV = 28.0;
    private static final int SIDE_DETECT_THRESHOLD = 100;

    private static final double SERVO_MAX = 1.0; // up
    private static final double SERVO_MIN = 0.0; // down during shooting

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Hardware
    private DcMotor intake;
    private DcMotorEx rightShooter;
    private DcMotorEx middleShooter;
    private DcMotorEx leftShooter;

    private Servo leftServo;
    private Servo middleServo;
    private Servo rightServo;

    private CRServo sorterServo;
    private CRServo sorterServo2;
    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;

    // PID state for each shooter
    private double right_integral = 0.0;
    private double right_lastError = 0.0;
    private double middle_integral = 0.0;
    private double middle_lastError = 0.0;
    private double left_integral = 0.0;
    private double left_lastError = 0.0;

    private boolean waiting = false;
    private double waitStartTime = 0;
    private boolean pathStarted = false;
    private boolean shootersAtSpeed = false;
    private double lastLoopTime = 0.0;

    // Staggered shooter startup
    private boolean shootersStarting = false;
    private double shooterStartTime = 0;
    private static final double SHOOTER_START_DELAY = 0.05; // 50ms delay between each shooter

    // Individual shooter ready states (THESE WERE NEVER SET BEFORE → BUG)
    private boolean rightShooterReady = false;
    private boolean middleShooterReady = false;
    private boolean leftShooterReady = false;

    // Failsafe timeout
    private static final double SHOOTER_TIMEOUT = 4.0; // 4 seconds to reach speed or move on

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(-36)));

        paths = new Paths(follower);

        // ---------- INTAKE ----------
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- SHOOTER MOTORS ----------
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        middleShooter = hardwareMap.get(DcMotorEx.class, "middleShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");

        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        middleShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ---------- SERVOS ----------
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        middleServo = hardwareMap.get(Servo.class, "middleServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftServo.setPosition(SERVO_MAX);
        middleServo.setPosition(SERVO_MAX);
        rightServo.setPosition(SERVO_MAX);

        // ---------- SORTER + COLOR SENSORS ----------
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        pathState = 0;
        waiting = false;
        pathStarted = false;
        shootersAtSpeed = false;
        shootersStarting = false;
        rightShooterReady = false;
        middleShooterReady = false;
        leftShooterReady = false;
        lastLoopTime = 0.0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {
        follower.update();

        // Calculate dt for PID
        double now = getRuntime();
        double dt = now - lastLoopTime;
        if (dt <= 0) dt = 1e-6;
        lastLoopTime = now;

        // Update shooter PIDF control
        updateShooters(dt);

        autonomousPathUpdate();

        // Get current velocities for telemetry
        double rightRPM = rightShooter.getVelocity() * 60.0 / TICKS_PER_REV;
        double middleRPM = middleShooter.getVelocity() * 60.0 / TICKS_PER_REV;
        double leftRPM = leftShooter.getVelocity() * 60.0 / TICKS_PER_REV;

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Waiting", waiting);
        panelsTelemetry.debug("Shooters At Speed", shootersAtSpeed);
        panelsTelemetry.debug("Follower isBusy", follower.isBusy());
        panelsTelemetry.debug("Shooter RPMs (R/M/L)", String.format("%.0f / %.0f / %.0f", rightRPM, middleRPM, leftRPM));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading°", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void updateShooters(double dt) {
        // Handle staggered startup to prevent voltage drop
        if (shootersStarting) {
            double elapsedStartTime = getRuntime() - shooterStartTime;

            // Start right shooter immediately
            if (elapsedStartTime >= 0) {
                double rightTicksPerSec = rightShooter.getVelocity();
                double rightRPM = rightTicksPerSec * 60.0 / TICKS_PER_REV;

                double rightError = TARGET_RPM - rightRPM;
                right_integral += rightError * dt;
                double rightDerivative = (rightError - right_lastError) / dt;
                right_lastError = rightError;

                double rightP = SHOOTER_KP * rightError;
                double rightI = SHOOTER_KI * right_integral;
                double rightD = SHOOTER_KD * rightDerivative;
                double rightF = SHOOTER_KF * TARGET_RPM;
                double rightPower = Range.clip(rightP + rightI + rightD + rightF, 0.0, 1.0);
                rightShooter.setPower(rightPower);
            }

            // Start middle shooter after delay
            if (elapsedStartTime >= SHOOTER_START_DELAY) {
                double middleTicksPerSec = middleShooter.getVelocity();
                double middleRPM = middleTicksPerSec * 60.0 / TICKS_PER_REV;

                double middleError = TARGET_RPM - middleRPM;
                middle_integral += middleError * dt;
                double middleDerivative = (middleError - middle_lastError) / dt;
                middle_lastError = middleError;

                double middleP = SHOOTER_KP * middleError;
                double middleI = SHOOTER_KI * middle_integral;
                double middleD = SHOOTER_KD * middleDerivative;
                double middleF = SHOOTER_KF * TARGET_RPM;
                double middlePower = Range.clip(middleP + middleI + middleD + middleF, 0.0, 1.0);
                middleShooter.setPower(middlePower);
            }

            // Start left shooter after 2x delay
            if (elapsedStartTime >= SHOOTER_START_DELAY * 2) {
                double leftTicksPerSec = leftShooter.getVelocity();
                double leftRPM = leftTicksPerSec * 60.0 / TICKS_PER_REV;

                double leftError = TARGET_RPM - leftRPM;
                left_integral += leftError * dt;
                double leftDerivative = (leftError - left_lastError) / dt;
                left_lastError = leftError;

                double leftP = SHOOTER_KP * leftError;
                double leftI = SHOOTER_KI * left_integral;
                double leftD = SHOOTER_KD * leftDerivative;
                double leftF = SHOOTER_KF * TARGET_RPM;
                double leftPower = Range.clip(leftP + leftI + leftD + leftF, 0.0, 1.0);
                leftShooter.setPower(leftPower);

                // After all shooters have started, switch to normal mode
                shootersStarting = false;
            }
            return;
        }

        // Normal PIDF control for all shooters (after staggered startup)
        // Get current velocities
        double rightTicksPerSec = rightShooter.getVelocity();
        double middleTicksPerSec = middleShooter.getVelocity();
        double leftTicksPerSec = leftShooter.getVelocity();

        double rightRPM = rightTicksPerSec * 60.0 / TICKS_PER_REV;
        double middleRPM = middleTicksPerSec * 60.0 / TICKS_PER_REV;
        double leftRPM = leftTicksPerSec * 60.0 / TICKS_PER_REV;

        // FIXED: Actually update individual ready flags
        rightShooterReady = (rightRPM >= RPM_THRESHOLD);
        middleShooterReady = (middleRPM >= RPM_THRESHOLD);
        leftShooterReady = (leftRPM >= RPM_THRESHOLD);

        // Check if all shooters are at speed
        shootersAtSpeed = rightShooterReady && middleShooterReady && leftShooterReady;

        // RIGHT shooter PIDF
        double rightError = TARGET_RPM - rightRPM;
        right_integral += rightError * dt;
        double rightDerivative = (rightError - right_lastError) / dt;
        right_lastError = rightError;

        double rightP = SHOOTER_KP * rightError;
        double rightI = SHOOTER_KI * right_integral;
        double rightD = SHOOTER_KD * rightDerivative;
        double rightF = SHOOTER_KF * TARGET_RPM;
        double rightPower = Range.clip(rightP + rightI + rightD + rightF, 0.0, 1.0);

        // MIDDLE shooter PIDF
        double middleError = TARGET_RPM - middleRPM;
        middle_integral += middleError * dt;
        double middleDerivative = (middleError - middle_lastError) / dt;
        middle_lastError = middleError;

        double middleP = SHOOTER_KP * middleError;
        double middleI = SHOOTER_KI * middle_integral;
        double middleD = SHOOTER_KD * middleDerivative;
        double middleF = SHOOTER_KF * TARGET_RPM;
        double middlePower = Range.clip(middleP + middleI + middleD + middleF, 0.0, 1.0);

        // LEFT shooter PIDF
        double leftError = TARGET_RPM - leftRPM;
        left_integral += leftError * dt;
        double leftDerivative = (leftError - left_lastError) / dt;
        left_lastError = leftError;

        double leftP = SHOOTER_KP * leftError;
        double leftI = SHOOTER_KI * left_integral;
        double leftD = SHOOTER_KD * leftDerivative;
        double leftF = SHOOTER_KF * TARGET_RPM;
        double leftPower = Range.clip(leftP + leftI + leftD + leftF, 0.0, 1.0);

        // Apply powers
        rightShooter.setPower(rightPower);
        middleShooter.setPower(middlePower);
        leftShooter.setPower(leftPower);
    }

    private void updateSorter() {
        // Read color sensors
        int lR = leftColor.red();
        int lG = leftColor.green();
        int lB = leftColor.blue();

        int rR = rightColor.red();
        int rG = rightColor.green();
        int rB = rightColor.blue();

        boolean leftOccupied = (lR > SIDE_DETECT_THRESHOLD) ||
                (lG > SIDE_DETECT_THRESHOLD) ||
                (lB > SIDE_DETECT_THRESHOLD);
        boolean rightOccupied = (rR > SIDE_DETECT_THRESHOLD) ||
                (rG > SIDE_DETECT_THRESHOLD) ||
                (rB > SIDE_DETECT_THRESHOLD);

        // Sorting logic (same as TeleOp)
        if (leftOccupied && rightOccupied) {
            sorterServo.setPower(0.0);
            sorterServo2.setPower(0.0);
        } else if (rightOccupied && !leftOccupied) {
            sorterServo.setPower(-1.0);
            sorterServo2.setPower(-1.0);// push to left
        } else if (leftOccupied && !rightOccupied) {
            sorterServo.setPower(1.0);
            sorterServo2.setPower(1.0);// push to right
        } else {
            sorterServo.setPower(1.0);
            sorterServo2.setPower(1.0);// default to right
        }
    }

    public static class Paths {

        public PathChain MovetoFirstShot;
        public PathChain MovetoPickup;
        public PathChain Pickup;
        public PathChain Movetosecondshot;
        public PathChain MovetoPickUpTwo;
        public PathChain PickUpTwo;
        public PathChain MovetoShootThree;
        public PathChain MovetoPickUpThree;
        public PathChain PickUpThree;
        public PathChain MovetoShootFour;

        public Paths(Follower follower) {
            MovetoFirstShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.369, 122.393), new Pose(95.721, 95.442))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-143))
                    .build();

            MovetoPickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.721, 95.442),
                                    new Pose(94.792, 94.281),
                                    new Pose(85.731, 87.311),
                                    new Pose(101.297, 82.664)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(0))
                    .build();

            Pickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.297, 82.664), new Pose(115.470, 81.502))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Movetosecondshot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(115.470, 81.502),
                                    new Pose(110.126, 94.048),
                                    new Pose(94.792, 94.745)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();

            MovetoPickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.792, 94.745),
                                    new Pose(77.135, 74.300),
                                    new Pose(74.347, 59.198),
                                    new Pose(92.933, 58.966)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            PickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.933, 58.966), new Pose(110.823, 58.501))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShootThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(110.823, 58.501),
                                    new Pose(70.165, 64.774),
                                    new Pose(95.024, 95.210)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();

            MovetoPickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.024, 95.210),
                                    new Pose(82.943, 55.713),
                                    new Pose(74.347, 36.197),
                                    new Pose(89.913, 34.571)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(0))
                    .build();

            PickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(89.913, 34.571), new Pose(116.399, 33.874))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            MovetoShootFour = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(116.399, 33.874),
                                    new Pose(134.753, 38.753),
                                    new Pose(76.438, 72.674),
                                    new Pose(94.792, 95.210)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();
        }
    }

    private void startShootingSequence() {
        waitStartTime = getRuntime();
        waiting = true;
        pathStarted = false;
        shootersAtSpeed = false;

        // Reset PID integrators
        right_integral = 0.0;
        middle_integral = 0.0;
        left_integral = 0.0;

        // Initialize staggered shooter startup
        shootersStarting = true;
        shooterStartTime = getRuntime();

        // Stop all shooters first to prepare for staggered startup
        rightShooter.setPower(0);
        middleShooter.setPower(0);
        leftShooter.setPower(0);

        // Servos stay up until shooters reach speed
        leftServo.setPosition(SERVO_MAX);
        middleServo.setPosition(SERVO_MAX);
        rightServo.setPosition(SERVO_MAX);

        // NOTE: Intake and sorter keep running from previous state
        // They are NOT stopped or started here
    }

    private void startIntakeSequence() {
        pathStarted = false;

        // Intake forward with sorting
        intake.setPower(1.0);
        // Sorter will be updated in main loop
    }

    private void stopRobot() {
        follower.setTeleOpDrive(0, 0, 0, true);
        intake.setPower(0);
        rightShooter.setPower(0);
        middleShooter.setPower(0);
        leftShooter.setPower(0);
        sorterServo.setPower(0);

        leftServo.setPosition(SERVO_MAX);
        middleServo.setPosition(SERVO_MAX);
        rightServo.setPosition(SERVO_MAX);
    }

    public void autonomousPathUpdate() {
        if (waiting) {
            // During shooting sequence, wait for shooters to reach speed
            // Keep intake and sorter running during entire shooting sequence

            // Check for timeout failsafe
            double shootingElapsed = getRuntime() - waitStartTime;
            boolean timedOut = shootingElapsed >= SHOOTER_TIMEOUT;

            // Lower each servo individually when its shooter reaches speed
            if (rightShooterReady) {
                rightServo.setPosition(SERVO_MIN);
            }
            if (middleShooterReady) {
                middleServo.setPosition(SERVO_MIN);
            }
            if (leftShooterReady) {
                leftServo.setPosition(SERVO_MIN);
            }

            // CRITICAL: Keep intake running to push balls through
            intake.setPower(1.0);

            // Continue shooting once all shooters are ready OR timeout occurs
            if (shootersAtSpeed || timedOut) {
                // Wait additional time from when shooters reached speed (or timed out) for balls to shoot
                if (shootingElapsed >= 3.0) {
                    waiting = false;
                    intake.setPower(0);
                    sorterServo.setPower(0);
                    sorterServo2.setPower(0);
                    rightShooter.setPower(0);
                    middleShooter.setPower(0);
                    leftShooter.setPower(0);

                    leftServo.setPosition(SERVO_MAX);
                    middleServo.setPosition(SERVO_MAX);
                    rightServo.setPosition(SERVO_MAX);

                    pathState++;
                }
            }
            return;
        }

        if (pathState > 9) {
            stopRobot();
            return;
        }

        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoFirstShot);
                    pathStarted = true;
                }
                else if (!follower.isBusy()) startShootingSequence();
                break;

            case 1:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickup);
                    pathStarted = true;
                }
                else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Pickup);
                    pathStarted = true;
                    intake.setPower(1.0);   //maybe try a lower power to slow ball down
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running, just advance to next path
                    pathState++;
                    pathStarted = false;
                }
                // Update sorter during intake
                else {
                    updateSorter();
                }
                break;

            case 3:
                if (!pathStarted) {
                    follower.followPath(paths.Movetosecondshot);
                    pathStarted = true;
                    // Intake and sorter continue running from previous state
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running during shooting sequence
                    startShootingSequence();
                }
                // Continue updating sorter during this movement
                else {
                    updateSorter();
                }
                break;

            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpTwo);
                    pathStarted = true;
                }
                else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.PickUpTwo);
                    pathStarted = true;
                    intake.setPower(1.0);
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running, just advance to next path
                    pathState++;
                    pathStarted = false;
                }
                // Update sorter during intake
                else {
                    updateSorter();
                }
                break;

            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoShootThree);
                    pathStarted = true;
                    // Intake and sorter continue running from previous state
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running during shooting sequence
                    startShootingSequence();
                }
                // Continue updating sorter during this movement
                else {
                    updateSorter();
                }
                break;

            case 7:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpThree);
                    pathStarted = true;
                }
                else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.PickUpThree);
                    pathStarted = true;
                    intake.setPower(1.0);
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running, just advance to next path
                    pathState++;
                    pathStarted = false;
                }
                // Update sorter during intake
                else {
                    updateSorter();
                }
                break;

            case 9:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoShootFour);
                    pathStarted = true;
                    // Intake and sorter continue running from previous state
                }
                else if (!follower.isBusy()) {
                    // Keep intake and sorter running during final shooting sequence
                    startShootingSequence();
                }
                // Continue updating sorter during this movement
                else {
                    updateSorter();
                }
                break;

        }
    }
}