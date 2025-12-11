package org.firstinspires.ftc.teamcode.pedroPathing.auto.Close;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths.RedFourCyclesPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StopRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateShooters;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.UpdateSorter;
import org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems.StartShootingSequence;

@Autonomous(name = "Red Close Auto", group = "Autonomous")
@Configurable
public class RedFourCycles extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private RedFourCyclesPaths.Paths paths;
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

    // Limelight and IMU
    private BNO055IMU imu;
    private Limelight3A limelight;

    private UpdateShooters shooterSubsystem;
    private UpdateSorter sorterSubsystem;
    private StartShootingSequence shootingSequence;
    private StopRobot stopRobotSubsystem;

    private double lastLoopTime = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126, 123, Math.toRadians(-143)));
        paths = new RedFourCyclesPaths.Paths(follower);

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

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0); // Switch to your AprilTag pipeline
        limelight.start(); // Start polling for data

        shooterSubsystem = new UpdateShooters(rightShooter, middleShooter, leftShooter);
        sorterSubsystem = new UpdateSorter(sorterServo, sorterServo2, leftColor, rightColor);
        shootingSequence = new StartShootingSequence(
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
        if (elapsedTime >= 28.0 && !timeoutTriggered && pathState < 10) {
            // Cancel any active shooting sequence or pause
            if (shootingSequence.isWaiting()) {
                shootingSequence.cancel();
            }
            isPausing = false;

            // Stop intake but DON'T stop shooters yet - let them coast down naturally
            intake.setPower(0);

            // Immediately start final path
            pathState = 10;
            pathStarted = false;
            timeoutTriggered = true;

            // Force start the final path immediately
            follower.followPath(paths.MovetoPickUpThree);
            pathStarted = true;
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
        panelsTelemetry.debug("Centering", shootingSequence.isCentering());
        panelsTelemetry.debug("Centered", shootingSequence.isCentered());
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
        // Don't process path updates if we've timed out and are executing the final path
        if (timeoutTriggered && pathState == 10) {
            // Just let the final path complete
            if (!follower.isBusy()) {
                shooterSubsystem.stopShooters();
                pathState++;
                pathStarted = false;
            }
            return;
        }

        if (shootingSequence.isWaiting() || isPausing) {
            return;
        }
        if (pathState > 10) {
            stopRobot();
            return;
        }

        switch (pathState) {
            case 0: // MOVE TO FIRST SHOT (and spin up shooters during movement - NO CENTERING YET)
                if (!pathStarted) {
                    follower.followPath(paths.MovetoFirstShot);
                    // Start spinning up shooters immediately
                    shooterSubsystem.startSideShoostersOnly(getRuntime());
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // No pause - go straight to centering and shooting
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 1: // FIRST SHOT - CENTER AND SHOOT (shooters already pre-spun)
                if (!pathStarted) {
                    // Use pre-spun shooters and DO center this time
                    shootingSequence.startWithPreSpunShooters(getRuntime());
                    pathStarted = true;
                }
                break;

            case 2: // MOVE TO FIRST PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.MovetoPickup);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // Advance to next state (previously commented out)
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 3: // FIRST PICKUP (after pause)
                if (!pathStarted) {
                    follower.setMaxPower(0.70);
                    follower.followPath(paths.Pickup);
                    intake.setPower(0.650);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 4: // MOVE TO SECOND SHOT
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.Movetosecondshot);
                    intake.setPower(0.65);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // Advance to next state (previously commented out)
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 5: // START SECOND SHOT (after pause) - WITH LIMELIGHT CENTERING
                if (!pathStarted) {
                    intake.setPower(0);
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 6: // MOVE TO SECOND PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.MovetoPickUpTwo);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 7: // SECOND PICKUP
                if (!pathStarted) {
                    follower.setMaxPower(0.75);
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
                    follower.setMaxPower(2);
                    follower.followPath(paths.MovetoShootThree);
                    intake.setPower(1.0);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    // Advance to next state (previously commented out)
                    pathState++;
                    pathStarted = false;
                }
                break;

            case 9: // START THIRD SHOT (after pause) - WITH LIMELIGHT CENTERING
                if (!pathStarted) {
                    intake.setPower(0);
                    shootingSequence.start(getRuntime());
                    pathStarted = true;
                }
                break;

            case 10: // FINAL MOVE
                if (!pathStarted) {
                    follower.setMaxPower(2);
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
