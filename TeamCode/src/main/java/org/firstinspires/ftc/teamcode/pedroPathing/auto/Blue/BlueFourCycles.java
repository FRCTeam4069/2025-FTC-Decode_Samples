package org.firstinspires.ftc.teamcode.pedroPathing.auto.Blue;


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
public class BlueFourCycles extends OpMode {

    // -----------------------------------------------------------------------
    // Shooter PIDF constants (matching TeleOp)
    // -----------------------------------------------------------------------
    private static final double SHOOTER_KP = 0.0002;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.0001;
    private static final double SHOOTER_KF = 0.0004;
    private static final double TARGET_RPM = 3000.0;
    private static final double RPM_THRESHOLD = 2500.0; // Lower servos when within 100 RPM

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
        // Get current velocities
        double rightTicksPerSec = rightShooter.getVelocity();
        double middleTicksPerSec = middleShooter.getVelocity();
        double leftTicksPerSec = leftShooter.getVelocity();

        double rightRPM = rightTicksPerSec * 60.0 / TICKS_PER_REV;
        double middleRPM = middleTicksPerSec * 60.0 / TICKS_PER_REV;
        double leftRPM = leftTicksPerSec * 60.0 / TICKS_PER_REV;

        // Check if all shooters are at speed
        shootersAtSpeed = (rightRPM >= RPM_THRESHOLD) &&
                (middleRPM >= RPM_THRESHOLD) &&
                (leftRPM >= RPM_THRESHOLD);

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
                            new BezierLine(new Pose(21.000, 123.000), new Pose(47.613, 96.619))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-36))
                    .build();

            MovetoPickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.613, 96.619),
                                    new Pose(51.113, 91.957),
                                    new Pose(45.537, 83.593)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))
                    .build();

            Pickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.537, 83.593), new Pose(29.739, 83.593))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Movetosecondshot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(29.739, 83.593),
                                    new Pose(34.839, 94.761),
                                    new Pose(47.861, 96.604)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            MovetoPickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.861, 96.604),
                                    new Pose(58.316, 88.240),
                                    new Pose(70.862, 59.895),
                                    new Pose(44.594, 58.065)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            PickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.594, 58.065), new Pose(30.658, 58.529))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShootThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30.658, 58.529),
                                    new Pose(42.968, 67.819),
                                    new Pose(65.265, 52.723),
                                    new Pose(47.396, 96.604)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            MovetoPickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.396, 96.604),
                                    new Pose(70.606, 67.819),
                                    new Pose(62.710, 33.213),
                                    new Pose(39.019, 34.839)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();

            PickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.019, 34.839), new Pose(27.639, 34.606))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MovetoShootFour = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(27.639, 34.606),
                                    new Pose(28.103, 68.516),
                                    new Pose(47.381, 96.619)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
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

        // Start spinning up shooters (PIDF will handle in loop)
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
            if (shootersAtSpeed) {
                // Lower servos once at speed
                leftServo.setPosition(SERVO_MIN);
                middleServo.setPosition(SERVO_MIN);
                rightServo.setPosition(SERVO_MIN);

                // CRITICAL: Keep intake running at 1.0 to push balls through
                intake.setPower(1.0);

                // Wait 3 seconds total from start for balls to shoot
                if (getRuntime() - waitStartTime >= 3.0) {
                    waiting = false;
                    intake.setPower(0);
                    sorterServo.setPower(0);
                    rightShooter.setPower(0);
                    middleShooter.setPower(0);
                    leftShooter.setPower(0);

                    leftServo.setPosition(SERVO_MAX);
                    middleServo.setPosition(SERVO_MAX);
                    rightServo.setPosition(SERVO_MAX);

                    pathState++;
                }
            } else {
                // Even before shooters are at speed, ensure intake is running
                intake.setPower(1.0);
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