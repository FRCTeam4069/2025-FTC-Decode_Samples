package org.firstinspires.ftc.teamcode.pedroPathing.auto.Close;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths.BlueFourCyclesPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StopRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateShooters;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateSorter;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StartShootingSequence;

@Autonomous(name = "Blue Close Four Cycles", group = "Autonomous")
@Configurable
public class BlueFourCycles extends OpMode {

    // Telemetry
    private TelemetryManager panelsTelemetry;

    // Pathing
    private Follower follower;
    private BlueFourCyclesPaths.Paths paths;
    private int pathState = 0;
    private boolean pathStarted = false;

    // Hardware
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

    // Subsystems
    private UpdateShooters shooterSubsystem;
    private UpdateSorter sorterSubsystem;
    private StartShootingSequence shootingSequence;
    private StopRobot stopRobotSubsystem;


    // Timing
    private double lastLoopTime = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Path / follower init
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(-36)));
        paths = new BlueFourCyclesPaths.Paths(follower);

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
        leftPushServo = hardwareMap.get(Servo.class, "leftPushServo");
        rightPushServo = hardwareMap.get(Servo.class, "rightPushServo");

        rightPushServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);

        // ---------- SORTER + COLOR SENSORS ----------
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        // ---------- Subsystem construction ----------
        shooterSubsystem = new UpdateShooters(rightShooter, middleShooter, leftShooter);
        sorterSubsystem = new UpdateSorter(sorterServo, sorterServo2, leftColor, rightColor);
        shootingSequence = new StartShootingSequence(
                shooterSubsystem,
                intake,
                leftServo, middleServo, rightServo,
                leftPushServo, rightPushServo
        );

        pathState = 0;
        pathStarted = false;
        lastLoopTime = 0.0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        stopRobotSubsystem = new StopRobot(
                follower,
                intake,
                rightShooter, middleShooter, leftShooter,
                sorterServo, sorterServo2,
                leftServo, middleServo, rightServo,
                leftPushServo, rightPushServo
        );

    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {

        // Update follower
        follower.update();

        // dt for PID
        double now = getRuntime();
        double dt = now - lastLoopTime;
        if (dt <= 0) dt = 1e-6;
        lastLoopTime = now;

        // Always update shooter PID (keeps state & readiness up-to-date)
        shooterSubsystem.updateShooters(dt, now);

        // If shooting sequence active, update it (it handles servos/intake/shooter end)
        if (shootingSequence.isWaiting()) {
            boolean finished = shootingSequence.update(now);
            if (finished) {
                // sequence finished → advance path
                pathState++;
                pathStarted = false;
            }
        }

        // Update sorter during intake motions and movement when not in shooting wait
        // (some states call sorter during follower busy)
        // We'll call sorter every loop unless shooting is active
        if (!shootingSequence.isWaiting()) {
            sorterSubsystem.updateSorter();
        }

        // Path & auto state machine
        autonomousPathUpdate();

        // Telemetry shooter RPMs
        double rightRPM = rightShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;
        double middleRPM = middleShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;
        double leftRPM = leftShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Waiting (shooting)", shootingSequence.isWaiting());
        panelsTelemetry.debug("Shooter RPMs (R/M/L)", String.format("%.0f / %.0f / %.0f", rightRPM, middleRPM, leftRPM));
        panelsTelemetry.debug("Follower isBusy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading°", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void startShootingSequence() {
        shooterSubsystem.stopShooters();
        shootingSequence.start(getRuntime());
        pathStarted = false;    }

    private void stopRobot() {
        stopRobotSubsystem.stopAll();      }

    public void autonomousPathUpdate() {
        if (shootingSequence.isWaiting()) {
            return;     }
        if (pathState > 7) {
            stopRobot();
            return;     }
        switch (pathState) {

            // ---------- 0 : MOVE TO FIRST SHOT ----------
            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoFirstShot);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // begin shooting sequence (staggered startup etc)
                    startShootingSequence();
                }
                break;

            // ---------- 1 : MOVE TO FIRST PICKUP ----------
            case 1:
                if (!pathStarted) {
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.MovetoPickup);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            // ---------- 2 : FIRST PICKUP ----------
            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Pickup);
                    pathStarted = true;
                    intake.setPower(1.0);
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    pathState++;
                    pathStarted = false;
                } else {
                    // During pickup while moving, keep updating sorter
                    sorterSubsystem.updateSorter();
                }
                break;

            // ---------- 3 : MOVE TO SECOND SHOT ----------
            case 3:
                if (!pathStarted) {
                    follower.followPath(paths.Movetosecondshot);
                    pathStarted = true;
                    intake.setPower(1.0);
                } else if (!follower.isBusy()) {
                    // stop intake/sorter and start shooting
                    startShootingSequence();
                } else {
                    sorterSubsystem.updateSorter();
                }
                break;

            // ---------- 4 : MOVE TO SECOND PICKUP ----------
            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpTwo);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            // ---------- 5 : SECOND PICKUP ----------
            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.PickUpTwo);
                    pathStarted = true;
                    intake.setPower(1.0);
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    pathState++;
                    pathStarted = false;
                } else {
                    sorterSubsystem.updateSorter();
                }
                break;

            // ---------- 6 : MOVE TO THIRD SHOT ----------
            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoShootThree);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    startShootingSequence();
                } else {
                    sorterSubsystem.updateSorter();
                }
                break;

            // ---------- 7 : FINAL MOVE / SHUTDOWN ----------
            case 7:
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpThree);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // Stop shooters and finish
                    shooterSubsystem.stopShooters();
                    pathState++;
                    pathStarted = false;
                }
                break;
        }
    }
}
