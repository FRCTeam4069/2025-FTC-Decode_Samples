package org.firstinspires.ftc.teamcode.pedroPathing.tele;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.BlockerServos;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.DrivebaseControl;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.LimelightCentering;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.PusherServos;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.ShooterControls;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.SorterControls;
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.TelemetryHelper;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Working TeleOp", group = "TeleOp")
public class TeleOpLimelightLocking extends LinearOpMode {

    // Subsystems
    private Follower follower;
    private TelemetryManager telemetryM;

    private DrivebaseControl drivebase;
    private LimelightCentering limelight;
    private ShooterControls shooter;
    private ShooterInterpolation interpolation;
    private BlockerServos blockerServos;
    private PusherServos pusherServos;
    private SorterControls sorterControls;
    private TelemetryHelper telemetryHelper;

    // Toggle state tracking
    private boolean dpadLeftReady = true;
    private boolean dpadRightReady = true;
    private boolean dpadUpReady = true;
    private boolean rightBumperReady = true;
    private boolean rightStickReady = true;

    // Vibration state
    private boolean wasAtTargetRPM = false;

    // Timing
    private double lastLoopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- INITIALIZE FOLLOWER ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // ---------- INITIALIZE IMU ----------
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);

        while (!imu.isGyroCalibrated() && !isStopRequested()) {
            telemetry.addLine("Calibrating IMU...");
            telemetry.update();
            sleep(10);
        }
        telemetry.addLine(imu.isGyroCalibrated() ? "IMU Ready" : "IMU Calibration Failed");

        // ---------- INITIALIZE LIMELIGHT ----------
        Limelight3A limelightHardware = hardwareMap.get(Limelight3A.class, "Limelight");
        limelightHardware.setPollRateHz(100);
        limelightHardware.pipelineSwitch(0);
        limelightHardware.start();

        // ---------- INITIALIZE HARDWARE ----------
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        CRServo sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        CRServo sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        RevColorSensorV3 leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        RevColorSensorV3 rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo middleServo = hardwareMap.get(Servo.class, "middleServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");
        Servo leftPushServo = hardwareMap.get(Servo.class, "leftPushServo");
        Servo rightPushServo = hardwareMap.get(Servo.class, "rightPushServo");

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        DcMotorEx middleShooter = hardwareMap.get(DcMotorEx.class, "middleShooter");
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");

        // ---------- INITIALIZE SUBSYSTEMS ----------
        drivebase = new DrivebaseControl(follower, imu);
        limelight = new LimelightCentering(limelightHardware);
        shooter = new ShooterControls(rightShooter, middleShooter, leftShooter);
        interpolation = new ShooterInterpolation();
        blockerServos = new BlockerServos(leftServo, middleServo, rightServo);
        pusherServos = new PusherServos(leftPushServo, rightPushServo);
        sorterControls = new SorterControls(intake, sorterServo, sorterServo2, leftColor, rightColor);
        telemetryHelper = new TelemetryHelper(telemetry);

        // ---------- READY ----------
        telemetry.addLine("TeleOp Triple Shooter Ready (Refactored)");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive(true);

        lastLoopTime = getRuntime();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.update();

            // Calculate delta time
            double now = getRuntime();
            double dt = now - lastLoopTime;
            if (dt <= 0) dt = 1e-6;
            lastLoopTime = now;

            // ---------- UPDATE LIMELIGHT ----------
            limelight.update();

            // ---------- SORTER CONTROL ----------
            // Right stick click toggles sorter on/off
            if (gamepad2.right_stick_button && rightStickReady) {
                sorterControls.toggleSorter();
                rightStickReady = false;
            }
            if (!gamepad2.right_stick_button) {
                rightStickReady = true;
            }

            sorterControls.updateSorter(gamepad2.left_stick_x);

            // ---------- CONTROL FLAGS ----------
            boolean centeringGamepad1 = gamepad1.right_trigger > RobotConstants.TRIGGER_THRESHOLD;
            boolean centeringGamepad2 = gamepad2.right_trigger > RobotConstants.TRIGGER_THRESHOLD;

            // ---------- INTERPOLATION SHOOTING SEQUENCE ----------
            if (centeringGamepad2) {
                if (!interpolation.isInterpolationShooting()) {
                    // Start interpolation sequence
                    interpolation.startInterpolationSequence(limelight.getDistanceInches());
                }

                // Manage servo positions based on shooter state
                if (!interpolation.areServosLowered()) {
                    // Keep servos up until shooters reach target
                    blockerServos.setAllUp();
                    pusherServos.setAllUp();

                    // Check if shooters reached target
                    double targetRPM = interpolation.getInterpolatedTargetRPM();
                    if (shooter.areAllEnabledShootersAtTarget(targetRPM, targetRPM, targetRPM)) {
                        interpolation.setServosLowered(now);
                    }
                } else {
                    // Shooters at target - lower servos with delay
                    blockerServos.setAllDown();

                    if (interpolation.shouldLowerPushServos(now)) {
                        pusherServos.setAllDown();
                    } else {
                        pusherServos.setAllUp();
                    }
                }
            } else {
                // Trigger released - reset
                interpolation.stopInterpolationSequence();
                blockerServos.setAllUp();
                pusherServos.setAllUp();

                // Manual intake control
                if (gamepad2.a) sorterControls.setIntakePower(1.0);
                else if (gamepad2.b) sorterControls.setIntakePower(-1.0);
                else sorterControls.setIntakePower(0.0);
            }

            // ---------- SHOOTER TOGGLES (gamepad2) ----------
            // Left trigger turns off all shooters
            if (gamepad2.left_trigger > RobotConstants.TRIGGER_THRESHOLD) {
                shooter.disableAllShooters();
            }

            // D-pad controls individual shooters
            if (gamepad2.dpad_left && dpadLeftReady) {
                shooter.setLeftShooterEnabled(!shooter.isLeftShooterEnabled());
                dpadLeftReady = false;
            }
            if (!gamepad2.dpad_left) dpadLeftReady = true;

            if (gamepad2.dpad_right && dpadRightReady) {
                shooter.setRightShooterEnabled(!shooter.isRightShooterEnabled());
                dpadRightReady = false;
            }
            if (!gamepad2.dpad_right) dpadRightReady = true;

            if (gamepad2.dpad_up && dpadUpReady) {
                shooter.setMiddleShooterEnabled(!shooter.isMiddleShooterEnabled());
                dpadUpReady = false;
            }
            if (!gamepad2.dpad_up) dpadUpReady = true;

            // ---------- SLOW MODE TOGGLE ----------
            if (gamepad1.right_bumper && rightBumperReady) {
                drivebase.toggleSlowMode();
                rightBumperReady = false;
            }
            if (!gamepad1.right_bumper) {
                rightBumperReady = true;
            }

            // ---------- UPDATE SHOOTER MOTORS ----------
            // Determine target RPMs based on mode
            double rightTarget, middleTarget, leftTarget;
            if (interpolation.isInterpolationShooting()) {
                double interpRPM = interpolation.getInterpolatedTargetRPM();
                rightTarget = interpRPM;
                middleTarget = interpRPM;
                leftTarget = interpRPM;
            } else {
                rightTarget = RobotConstants.right_targetRPM;
                middleTarget = RobotConstants.middle_targetRPM;
                leftTarget = RobotConstants.left_targetRPM;
            }

            shooter.update(dt, rightTarget, middleTarget, leftTarget);

            // ---------- VIBRATION FEEDBACK ----------
            boolean allAtTarget = shooter.areAllEnabledShootersAtTarget(rightTarget, middleTarget, leftTarget);
            boolean anyEnabled = shooter.isAnyShooterEnabled();

            if (allAtTarget && !wasAtTargetRPM && anyEnabled) {
                gamepad2.rumble(RobotConstants.vibration_duration_ms);
            }
            wasAtTargetRPM = allAtTarget && anyEnabled;

            // ---------- DRIVING CONTROL ----------
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (centeringGamepad1 && limelight.isTagVisible()) {
                // Rotation lock with Limelight
                double rotateCorrection = limelight.calculateRotationCorrection(dt);
                drivebase.updateDriveWithRotationOverride(y, x, rotateCorrection);
            } else {
                // Normal driving
                drivebase.updateDrive(y, x, rx, false);
            }

            // ---------- RESET HEADING ----------
            if (gamepad1.x) {
                drivebase.resetHeading();
            }

            // ---------- TELEMETRY ----------
            telemetryHelper.displayAll(sorterControls, drivebase, shooter, interpolation,
                    limelight, centeringGamepad1,
                    rightTarget, middleTarget, leftTarget);
        }
    }
}