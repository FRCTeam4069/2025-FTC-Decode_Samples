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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths.BlueFourCyclesPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StopRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateShooters;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateSorter;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StartShootingSequence;

@Autonomous(name = "Blue Close Auto", group = "Autonomous")
@Configurable
public class BlueFourCycles extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private BlueFourCyclesPaths.Paths paths;
    private int pathState = 0;
    private boolean pathStarted = false;

    // Pause timers
    private boolean isPausing = false;
    private double pauseStartTime = 0.0;
    private double pauseDuration = 0.0;

    // Auto timeout
    private static final double AUTO_TIMEOUT = 29.0;
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

    private UpdateShooters shooterSubsystem;
    private UpdateSorter sorterSubsystem;
    private StartShootingSequence shootingSequence;
    private StopRobot stopRobotSubsystem;

    private double lastLoopTime = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(-36)));
        paths = new BlueFourCyclesPaths.Paths(follower);

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

        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);

        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        shooterSubsystem = new UpdateShooters(rightShooter, middleShooter, leftShooter);
        sorterSubsystem = new UpdateSorter(sorterServo, sorterServo2, leftColor, rightColor);
        shootingSequence = new StartShootingSequence(
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
        if (elapsedTime >= AUTO_TIMEOUT && !timeoutTriggered && pathState < 10) {
            // Cancel any active shooting sequence or pause
            shootingSequence.cancel();
            isPausing = false;

            // Stop intake and jump to final path
            intake.setPower(0);
            shooterSubsystem.stopShooters();

            // Jump to final path state
            pathState = 10;
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
        double rightRPM = rightShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;
        double middleRPM = middleShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;
        double leftRPM = leftShooter.getVelocity() * 60.0 / BlueFourCyclesConstants.TICKS_PER_REV;

        panelsTelemetry.debug("Auto Time", String.format("%.1f / %.1f", elapsedTime, AUTO_TIMEOUT));
        panelsTelemetry.debug("Timeout Triggered", timeoutTriggered);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Pausing", isPausing);
        panelsTelemetry.debug("Waiting (shooting)", shootingSequence.isWaiting());
        panelsTelemetry.debug("Shooters at speed", shooterSubsystem.shootersAtSpeed);
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
        if (pathState > 10) {
            stopRobot();
            return;
        }

        switch (pathState) {
            case 0: // MOVE TO FIRST SHOT (and spin up shooters during movement)
                if (!pathStarted) {
                    follower.setMaxPower(2);
                    follower.followPath(paths.MovetoFirstShot);
                    // Start spinning up shooters immediately
                    shooterSubsystem.startSideShoostersOnly(getRuntime());
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // 0.5 second pause before shooting
                    startPause(0.50);
                }
                break;

            case 1: // START FIRST SHOT (after pause)
                if (!pathStarted) {
                    shootingSequence.startWithPreSpunShooters(getRuntime());
                    pathStarted = true;
                }
                break;

            case 2: // MOVE TO FIRST PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(2);
                    follower.followPath(paths.MovetoPickup);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // 0.5 second pause before intake path
                    startPause(0.5);
                }
                break;

            case 3: // FIRST PICKUP (after pause)
                if (!pathStarted) {
                    follower.setMaxPower(0.46);  //was 0.42
                    follower.followPath(paths.Pickup);
                    intake.setPower(0.650); //was 1
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(0.650);
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 4: // MOVE TO SECOND SHOT
                if (!pathStarted) {
                    follower.setMaxPower(2);
                    follower.followPath(paths.Movetosecondshot);
                    intake.setPower(0.750);  //was 1
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // 0.5 second pause before second shot
                    startPause(0.50);
                }
                break;

            case 5: // START SECOND SHOT (after pause)
                if (!pathStarted) {
                    intake.setPower(0);
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 6: // MOVE TO SECOND PICKUP
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpTwo);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 7: // SECOND PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(0.46);
                    follower.followPath(paths.PickUpTwo);
                    intake.setPower(0.650);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 8: // MOVE TO THIRD SHOT
                if (!pathStarted) {
                    follower.followPath(paths.MovetoShootThree);
                    intake.setPower(1.0);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // 0.5 second pause before third shot
                    startPause(0.50);
                }
                break;

            case 9: // START THIRD SHOT (after pause)
                if (!pathStarted) {
                    intake.setPower(0);
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 10: // FINAL MOVE
                if (!pathStarted) {
                    follower.followPath(paths.MovetoPickUpThree);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    shooterSubsystem.stopShooters();
                    pathState++;
                    pathStarted = false;
                }
                break;
        }
    }
}