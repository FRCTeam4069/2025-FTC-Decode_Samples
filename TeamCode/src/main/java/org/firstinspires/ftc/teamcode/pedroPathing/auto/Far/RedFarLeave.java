package org.firstinspires.ftc.teamcode.pedroPathing.auto.Far;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueAutoFarConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.FarShootingSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StopRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateShootersFar;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateSorter;

@Autonomous(name = "Red Far Auto", group = "Autonomous")
@Configurable
public class RedFarLeave extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private boolean pathStarted = false;
    private Paths paths;

    // --- TUNE THIS VARIABLE IN DASHBOARD ---
    public static double LIMELIGHT_OFFSET = 0.0;

    // Pause timers
    private boolean isPausing = false;
    private double pauseStartTime = 0.0;
    private double pauseDuration = 0.0;

    // --- Path Timeout Logic ---
    private static final double PATH_TIMEOUT_SECONDS = 5.0;
    private double pathStartTime = 0.0;

    // Auto timeout
    private static final double AUTO_TIMEOUT = 28.0;
    private double autoStartTime = 0.0;
    private boolean timeoutTriggered = false;

    private DcMotor intake;
    private DcMotorEx rightShooter;
    private DcMotorEx middleShooter;
    private DcMotorEx leftShooter;

    private Servo leftServo;
    private Servo middleServo;
    private Servo rightServo;
    private Servo leftPushServo;
    private Servo rightPushServo;

    private CRServo sorterServo;
    private CRServo sorterServo2;

    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;

    private BNO055IMU imu;
    private Limelight3A limelight;

    private UpdateShootersFar shooterSubsystem;
    private UpdateSorter sorterSubsystem;
    private FarShootingSequence shootingSequence;
    private StopRobot stopRobotSubsystem;

    private double lastLoopTime = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // MIRRORED START: Y = 144 - 9.058 = 134.942 | Angle = -90
        follower.setStartingPose(new Pose(53.187, 134.942, Math.toRadians(-90)));

        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        middleServo = hardwareMap.get(Servo.class, "middleServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftPushServo = hardwareMap.get(Servo.class, "leftPushServo");
        rightPushServo = hardwareMap.get(Servo.class, "rightPushServo");

        rightPushServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        middleServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueAutoFarConstants.SERVO_MAX);

        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        shooterSubsystem = new UpdateShootersFar(rightShooter, middleShooter, leftShooter);
        sorterSubsystem = new UpdateSorter(sorterServo, sorterServo2, leftColor, rightColor);
        shootingSequence = new FarShootingSequence(
                shooterSubsystem,
                leftServo, middleServo, rightServo,
                leftPushServo, rightPushServo,
                sorterServo, sorterServo2,
                leftColor,
                follower,
                imu,
                limelight
        );

        stopRobotSubsystem = new StopRobot(
                follower,
                intake,
                rightShooter, middleShooter, leftShooter,
                sorterServo, sorterServo2,
                leftServo, middleServo, rightServo,
                leftPushServo, rightPushServo
        );

        pathState = 0;
        pathStarted = false;
        isPausing = false;
        pauseStartTime = 0.0;
        pauseDuration = 0.0;
        lastLoopTime = 0.0;
        timeoutTriggered = false;

        panelsTelemetry.debug("Status", "Initialized Red Far");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();
        autoStartTime = getRuntime();
    }

    @Override
    public void loop() {
        follower.update();

        double now = getRuntime();
        double dt = now - lastLoopTime;
        if (dt <= 0) dt = 1e-6;
        lastLoopTime = now;

        // Auto Timeout Check (28s)
        double elapsedTime = now - autoStartTime;
        if (elapsedTime >= 28.0 && !timeoutTriggered && pathState < 6) {
            if (shootingSequence.isWaiting()) {
                shootingSequence.cancel();
            }
            isPausing = false;
            intake.setPower(0);
            pathState = 6;
            pathStarted = false;
            timeoutTriggered = true;
            follower.followPath(paths.Leave);
            pathStarted = true;
        }

        shooterSubsystem.updateShooters(dt, now);

        // Update sequence
        if (shootingSequence.isWaiting()) {
            boolean finished = shootingSequence.update(now);
            if (finished) {
                pathState++;
                pathStarted = false;
            }
        }

        if (isPausing) {
            if (now - pauseStartTime >= pauseDuration) {
                isPausing = false;
                pathState++;
                pathStarted = false;
            }
        }

        if (!shootingSequence.isWaiting() && !isPausing) {
            sorterSubsystem.updateSorter();
        }

        autonomousPathUpdate();

        double rightRPM = rightShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;
        double middleRPM = middleShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;
        double leftRPM = leftShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;

        panelsTelemetry.debug("Red Auto Time", String.format("%.1f / %.1f", elapsedTime, AUTO_TIMEOUT));
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("LL Offset", LIMELIGHT_OFFSET);
        panelsTelemetry.debug("Shooter RPMs", String.format("%.0f/%.0f/%.0f", rightRPM, middleRPM, leftRPM));
        panelsTelemetry.update(telemetry);
    }

    private void startPause(double duration) {
        isPausing = true;
        pauseStartTime = getRuntime();
        pauseDuration = duration;
    }

    private void stopRobot() {
        stopRobotSubsystem.stopAll();
    }

    private boolean isPathFinishedOrTimedOut() {
        double pathElapsed = getRuntime() - pathStartTime;
        boolean isTimedOut = pathElapsed >= PATH_TIMEOUT_SECONDS;

        if (isTimedOut && follower.isBusy()) {
            follower.breakFollowing();
            panelsTelemetry.debug("Path Warning", "TIMEOUT TRIGGERED");
        }

        return !follower.isBusy() || isTimedOut;
    }

    public void autonomousPathUpdate() {
        if (timeoutTriggered && pathState == 6) {
            if (!follower.isBusy()) {
                shooterSubsystem.stopShooters();
                intake.setPower(0);
                pathState++;
                pathStarted = false;
            }
            return;
        }

        if (shootingSequence.isWaiting() || isPausing) {
            return;
        }
        if (pathState > 8) {
            stopRobot();
            return;
        }

        switch (pathState) {
            case 0: // MOVE TO FIRST SHOT
                if (!pathStarted) {
                    follower.setMaxPower(0.65);
                    follower.followPath(paths.FarShot);
                    pathStarted = true;
                    pathStartTime = getRuntime();
                } else if (isPathFinishedOrTimedOut()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 1: // SHOOTING SEQUENCE
                if (!pathStarted) {
                    follower.breakFollowing();
                    //shootingSequence.setOffset(LIMELIGHT_OFFSET); // Pass tunable offset
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 2: // MOVE TO PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.MoveToPickUp);
                    pathStarted = true;
                    pathStartTime = getRuntime();
                } else if (isPathFinishedOrTimedOut()) {
                    startPause(0.5);
                }
                break;

            case 3: // PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.PickUp);
                    intake.setPower(0.650);
                    pathStarted = true;
                    pathStartTime = getRuntime();
                } else if (isPathFinishedOrTimedOut()) {
                    follower.setMaxPower(0.750);
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 4: // MOVE TO SECOND SHOT
                if (!pathStarted) {
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.MovetoFarShotTwo);
                    intake.setPower(0.60);
                    pathStarted = true;
                    pathStartTime = getRuntime();
                } else if (isPathFinishedOrTimedOut()) {
                    intake.setPower(0.0);
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 5: // SHOOTING SEQUENCE 2
                if (!pathStarted) {
                    follower.breakFollowing();
                   // shootingSequence.setOffset(LIMELIGHT_OFFSET); // Pass tunable offset
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 6: // LEAVE
                if (!pathStarted) {
                    follower.followPath(paths.Leave);
                    pathStarted = true;
                    pathStartTime = getRuntime();
                } else if (isPathFinishedOrTimedOut()) {
                    shooterSubsystem.stopShooters();
                    intake.setPower(0);
                    pathState++;
                    pathStarted = false;
                }
                break;
        }
    }

    public static class Paths {

        public PathChain FarShot;
        public PathChain MoveToPickUp;
        public PathChain PickUp;
        public PathChain MovetoFarShotTwo;
        public PathChain Leave;

        public Paths(Follower follower) {
            // MIRRORED COORDINATES (Red): Y = 144 - Blue_Y, Heading = -Blue_Heading

            FarShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.187, 134.942),
                                    new Pose(58.083, 129.410),
                                    new Pose(63.000, 132.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(61))
                    .build();

            MoveToPickUp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.000, 132.000),
                                    new Pose(69.700, 120.116),
                                    new Pose(14.865, 124.955)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(180))
                    .build();

            PickUp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.865, 124.955),
                                    new Pose(8.129, 130.761),
                                    new Pose(17.187, 130.994),
                                    new Pose(17.884, 131.000),
                                    new Pose(9.290, 131.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MovetoFarShotTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.290, 139.355),
                                    new Pose(28.335, 122.865),
                                    new Pose(63.000, 132.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(67))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(63.000, 132.000),
                                    new Pose(57.000, 109.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(180))
                    .build();
        }
    }
}