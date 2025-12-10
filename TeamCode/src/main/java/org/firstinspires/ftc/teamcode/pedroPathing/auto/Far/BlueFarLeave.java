package org.firstinspires.ftc.teamcode.pedroPathing.auto.Far;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueAutoFarConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.FarShootingSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StopRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateShootersFar;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateSorter;

@Autonomous(name = "Blue Far Auto", group = "Autonomous")
@Configurable
public class BlueFarLeave extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private boolean pathStarted = false;
    private Paths paths;

    // Pause timers
    private boolean isPausing = false;
    private double pauseStartTime = 0.0;
    private double pauseDuration = 0.0;

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

    private UpdateShootersFar shooterSubsystem;
    private UpdateSorter sorterSubsystem;
    private FarShootingSequence shootingSequence;
    private StopRobot stopRobotSubsystem;

    private double lastLoopTime = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

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

        shooterSubsystem = new UpdateShootersFar(rightShooter, middleShooter, leftShooter);
        sorterSubsystem = new UpdateSorter(sorterServo, sorterServo2, leftColor, rightColor);
        shootingSequence = new FarShootingSequence(
                shooterSubsystem,
                leftServo, middleServo, rightServo,
                leftPushServo, rightPushServo,
                sorterServo, sorterServo2,
                leftColor
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

        panelsTelemetry.debug("Status", "Initialized");
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

        // Check for timeout (28 seconds since start)
        double elapsedTime = now - autoStartTime;
        if (elapsedTime >= AUTO_TIMEOUT && !timeoutTriggered && pathState < 6) {
            // Cancel any active shooting sequence or pause
            shootingSequence.cancel();
            isPausing = false;

            // Stop intake and jump to final path
            intake.setPower(0);
            shooterSubsystem.stopShooters();

            // Jump to final path state (Leave)
            pathState = 6;
            pathStarted = false;
            timeoutTriggered = true;
        }

        // Always update shooter PID
        shooterSubsystem.updateShooters(dt, now);

        // Update shooting sequence if active
        if (shootingSequence.isWaiting()) {
            boolean finished = shootingSequence.update(now);
            if (finished) {
                pathState++;
                pathStarted = false;
            }
        }

        // Handle pause timing
        if (isPausing) {
            if (now - pauseStartTime >= pauseDuration) {
                isPausing = false;
                pathState++;
                pathStarted = false;
            }
        }

        // Update sorter when not shooting or pausing
        if (!shootingSequence.isWaiting() && !isPausing) {
            sorterSubsystem.updateSorter();
        }

        autonomousPathUpdate();

        // Telemetry
        double rightRPM = rightShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;
        double middleRPM = middleShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;
        double leftRPM = leftShooter.getVelocity() * 60.0 / BlueAutoFarConstants.TICKS_PER_REV;

        panelsTelemetry.debug("Auto Time", String.format("%.1f / %.1f", elapsedTime, AUTO_TIMEOUT));
        panelsTelemetry.debug("Timeout Triggered", timeoutTriggered);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Pausing", isPausing);
        panelsTelemetry.debug("Waiting (shooting)", shootingSequence.isWaiting());
        panelsTelemetry.debug("Shooter RPMs (R/M/L)", String.format("%.0f / %.0f / %.0f", rightRPM, middleRPM, leftRPM));
        panelsTelemetry.debug("Left Color Green", leftColor.green());
        panelsTelemetry.debug("Follower isBusy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading°", Math.toDegrees(follower.getPose().getHeading()));
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

    public void autonomousPathUpdate() {
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
                } else if (!follower.isBusy()) {
                    // 1 second pause before shooting
                    startPause(1.0);
                }
                break;

            case 1: // START FIRST SHOT (after pause) - shoots all 3 balls one at a time
                if (!pathStarted) {
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 2: // MOVE TO PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.MoveToPickUp);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // 0.5 second pause before pickup
                    startPause(0.5);
                }
                break;

            case 3: // PICKUP (after pause)
                if (!pathStarted) {
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.PickUp);
                    intake.setPower(0.650);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
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
                } else if (!follower.isBusy()) {
                    // 1 second pause before second shot
                    intake.setPower(0.0);
                    startPause(1.0);
                }
                break;

            case 5: // START SECOND SHOT (after pause) - shoots all 3 balls one at a time
                if (!pathStarted) {
                    shootingSequence.start(getRuntime());
                    intake.setPower(0.0);
                    pathStarted = true;
                }
                break;

            case 6: // LEAVE
                if (!pathStarted) {
                    follower.followPath(paths.Leave);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
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
            FarShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(58.083, 14.590),
                                    new Pose(63.000, 12.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-64))
                    .build();

            MoveToPickUp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.000, 12.000),
                                    new Pose(69.700, 23.884),
                                    new Pose(55, 35.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-64), Math.toRadians(180))
                    .build();

            PickUp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55, 35.303), new Pose(21.368, 35.535))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MovetoFarShotTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.368, 35.535),
                                    new Pose(28.335, 21.135),
                                    new Pose(63.000, 12.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-65))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.000, 12.000), new Pose(57.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(180))
                    .build();
        }
    }
}