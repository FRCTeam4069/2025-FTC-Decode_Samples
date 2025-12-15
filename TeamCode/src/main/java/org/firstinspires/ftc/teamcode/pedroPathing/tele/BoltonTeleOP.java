package org.firstinspires.ftc.teamcode.pedroPathing.tele;

import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.*;

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
import org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems.RobotConstants;

@Configurable
@TeleOp(name = "Testing teleop", group = "TeleOp")
public class BoltonTeleOP extends LinearOpMode {

    // Subsystems
    private Follower follower;
    private TelemetryManager telemetryM;
    private DrivebaseControl drivebase;
    private AprilTagCentering aprilTagCentering;
    private ShooterControls shooter;
    private ShooterInterpolation interpolation;
    private BlockerServos blockerServos;
    private PusherServos pusherServos;
    private SorterControls sorterControls;
    private TelemetryHelper telemetryHelper;

    // Button debounce
    private boolean dpadLeftReady = true;
    private boolean dpadRightReady = true;
    private boolean dpadUpReady = true;
    private boolean rightBumperReady = true;
    private boolean rightStickReady = true;
    private boolean xButtonReady = true;

    private boolean wasAtTargetRPM = false;
    private double lastLoopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- FOLLOWER ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // ---------- IMU ----------
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

        // ---------- LIMELIGHT ----------
        Limelight3A limelightHardware = hardwareMap.get(Limelight3A.class, "Limelight");
        limelightHardware.setPollRateHz(100);
        limelightHardware.pipelineSwitch(0);
        limelightHardware.start();

        // ---------- HARDWARE ----------
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

        // ---------- SUBSYSTEMS ----------
        drivebase = new DrivebaseControl(follower, imu);
        aprilTagCentering = new AprilTagCentering(
                follower, imu, limelightHardware, gamepad1, gamepad2
        );
        shooter = new ShooterControls(rightShooter, middleShooter, leftShooter);
        interpolation = new ShooterInterpolation();
        blockerServos = new BlockerServos(leftServo, middleServo, rightServo);
        pusherServos = new PusherServos(leftPushServo, rightPushServo);
        sorterControls = new SorterControls(intake, sorterServo, sorterServo2, leftColor, rightColor);
        telemetryHelper = new TelemetryHelper(telemetry);

        telemetry.addLine("TeleOp Ready - AprilTag Centering Enabled");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive(true);
        lastLoopTime = getRuntime();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.update();

            double now = getRuntime();
            double dt = now - lastLoopTime;
            if (dt <= 0) dt = 1e-6;
            lastLoopTime = now;

            // ---------- SORTER ----------
            if (gamepad2.right_stick_button && rightStickReady) {
                sorterControls.toggleSorter();
                rightStickReady = false;
            }
            if (!gamepad2.right_stick_button) rightStickReady = true;

            if (gamepad2.x && xButtonReady) {
                sorterControls.toggleInwardSpin();
                xButtonReady = false;
            }
            if (!gamepad2.x) xButtonReady = true;

            sorterControls.updateSorter(gamepad2.left_stick_x);

            // ---------- INTERPOLATION SHOOTING ----------
            boolean interpolating = gamepad2.right_trigger > RobotConstants.TRIGGER_THRESHOLD;

            if (interpolating) {
                if (!interpolation.isInterpolationShooting()) {
                    //shooter.enableAllShooters();
                    interpolation.startInterpolationSequence(
                            aprilTagCentering.getDistanceInches(),
                            aprilTagCentering.isTagVisible()
                    );
                }
                sorterControls.setIntakePower(1.0);

                if (!interpolation.areServosLowered()) {
                    blockerServos.setAllUp();
                    pusherServos.setAllUp();
                    double targetRPM = interpolation.getInterpolatedTargetRPM();
                    if (shooter.areAllEnabledShootersAtTarget(
                            targetRPM, targetRPM, targetRPM)) {
                        interpolation.setServosLowered(now);
                    }
                } else {
                    blockerServos.setAllDown();
                    if (interpolation.shouldLowerPushServos(now)) {
                        pusherServos.setAllDown();
                    } else {
                        pusherServos.setAllUp();
                    }
                }
            } else {
                interpolation.stopInterpolationSequence();
                blockerServos.setAllUp();
                pusherServos.setAllUp();

                if (gamepad2.a) sorterControls.setIntakePower(1.0);
                else if (gamepad2.b) sorterControls.setIntakePower(-1.0);
                else sorterControls.setIntakePower(0.0);
            }

            // ---------- SHOOTER TOGGLES ----------
            if (gamepad2.left_trigger > RobotConstants.TRIGGER_THRESHOLD) {
                shooter.disableAllShooters();
            }

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

            // ---------- SLOW MODE ----------
            if (gamepad1.right_bumper && rightBumperReady) {
                drivebase.toggleSlowMode();
                rightBumperReady = false;
            }
            if (!gamepad1.right_bumper) rightBumperReady = true;

            // ---------- SHOOTER RPM ----------
            double rightTarget, middleTarget, leftTarget;
            if (interpolation.isInterpolationShooting()) {
                double rpm = interpolation.getInterpolatedTargetRPM();
                rightTarget = middleTarget = leftTarget = rpm;
            } else {
                rightTarget = RobotConstants.right_targetRPM;
                middleTarget = RobotConstants.middle_targetRPM;
                leftTarget = RobotConstants.left_targetRPM;
            }
            shooter.update(dt, rightTarget, middleTarget, leftTarget);

            // ---------- VIBRATION ----------
            boolean allAtTarget = shooter.areAllEnabledShootersAtTarget(
                    rightTarget, middleTarget, leftTarget);
            boolean anyEnabled = shooter.isAnyShooterEnabled();

            if (allAtTarget && !wasAtTargetRPM && anyEnabled) {
                gamepad2.rumble(RobotConstants.vibration_duration_ms);
            }
            wasAtTargetRPM = allAtTarget && anyEnabled;

            // ---------- APRIL TAG CENTERING ----------
            boolean centeringActive =
                    gamepad1.right_trigger > RobotConstants.TRIGGER_THRESHOLD;

            if (centeringActive) {
                // Auto-drive handled internally by AprilTagCentering
                aprilTagCentering.update(true, dt);
            } else {
                aprilTagCentering.update(false, dt);

                double y = -gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;
                drivebase.updateDrive(y, x, rx, true);
            }

            // ---------- RESET HEADING ----------
            if (gamepad1.x) {
                drivebase.resetHeading();
                aprilTagCentering.resetHeading();
            }

            // ---------- TELEMETRY ----------
            /*telemetryHelper.displayAll(
                    sorterControls, drivebase, shooter,
                    interpolation, AprilTagCentering, aprilTagCentering,
                    centeringActive, rightTarget, middleTarget, leftTarget
            ); */
        }
    }
}