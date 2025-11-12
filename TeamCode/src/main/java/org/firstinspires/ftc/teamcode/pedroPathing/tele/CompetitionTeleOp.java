package org.firstinspires.ftc.teamcode.pedroPathing.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private BNO055IMU imu;
    private double imuOffset = 0;
    private Limelight3A limelight;

    // === HARDWARE ===
    private DcMotor intakeMotor;
    private CRServo intakeServo;
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    private ColorSensor middleSensor;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private DcMotorEx middleShooter;

    // === CONSTANTS ===
    private static final double ROTATE_KP = 0.025; // Tuned lower to reduce oscillation/overshoot
    private static final double TX_DEADBAND = 2.0; // Increased deadband for stability
    private static final double MAX_ROTATE_SPEED = 0.6;
    private static final double TRIGGER_THRESHOLD = 0.3;
    private static final double TICKS_PER_REV = 28.0; // Assuming standard encoder ticks per revolution
    private static final double TARGET_RPM = 3000.0;

    private boolean slowMode = false;
    private boolean slowToggleAllowed = true;
    private double slowModeMultiplier = 0.25; // Quarter speed, Logan like half speed

    private boolean shootersRunning = false;
    private boolean leftOn = false;
    private boolean rightOn = false;
    private boolean middleOn = false;
    private double prevTrigger = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---------- FOLLOWER ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // ---------- IMU ----------
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        while (!imu.isGyroCalibrated() && !isStopRequested()) {
            telemetry.addLine("Calibrating IMU...");
            telemetry.update();
            sleep(10);
        }

        telemetry.addLine(imu.isGyroCalibrated() ? "IMU Ready" : "IMU Calibration Failed");

        // ---------- LIMELIGHT ----------
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---------- INTAKE ----------
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- SERVO ----------
        intakeServo = hardwareMap.get(CRServo.class, "motor67");

        // ---------- COLOR SENSORS ----------
        leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
        middleSensor = hardwareMap.get(ColorSensor.class, "middleSensor");

        // ---------- SHOOTER MOTORS ----------
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        middleShooter = hardwareMap.get(DcMotorEx.class, "middleShooter");

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("All systems ready: Limelight + Intake + Servo + Color Sensors + Shooters");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive(true);

        double servoPower = 0.0;

        while (opModeIsActive()) {
            follower.update();
            telemetryM.update();

            // --- LIMELIGHT ---
            LLResult result = limelight.getLatestResult();
            boolean tagVisible = false;
            int tagId = -1;
            double tx = 0.0;

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    tagVisible = true;
                    FiducialResult primary = fiducials.get(0);
                    tagId = primary.getFiducialId();
                    tx = primary.getTargetXDegrees();
                }
            }

            if (tagVisible) {
                telemetry.addData("Tag ID", tagId);
                telemetry.addData("tx (deg)", "%.2f", tx);
            } else {
                telemetry.addLine("No AprilTag");
            }

            // --- DRIVE INPUTS ---
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double heading = getHeading();
            double scale = slowMode ? slowModeMultiplier : 1.0;

            double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

            double currentTrigger = gamepad1.right_trigger;
            boolean centering = currentTrigger > TRIGGER_THRESHOLD;

            if (centering && tagVisible) {
                double rotate = 0.0;
                if (Math.abs(tx) > TX_DEADBAND) {
                    rotate = -tx * ROTATE_KP;
                    rotate = com.qualcomm.robotcore.util.Range.clip(rotate, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);
                }
                follower.setTeleOpDrive(0, 0, rotate, false);
            } else {
                follower.setTeleOpDrive(rotatedY * scale, rotatedX * scale, rx * scale, true);
            }

            // --- SHOOTER CONTROL ---
            if (prevTrigger <= TRIGGER_THRESHOLD && currentTrigger > TRIGGER_THRESHOLD) {
                if (shootersRunning) {
                    // Stop shooters
                    leftShooter.setVelocity(0);
                    middleShooter.setVelocity(0);
                    rightShooter.setVelocity(0);
                    shootersRunning = false;
                } else {
                    // Start shooters based on current sensor readings
                    leftOn = isBallDetected(leftSensor);
                    middleOn = isBallDetected(middleSensor);
                    rightOn = isBallDetected(rightSensor);
                    if (leftOn || middleOn || rightOn) {
                        shootersRunning = true;
                    }
                }
            }

            // Maintain shooter velocities if running
            double targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;
            if (shootersRunning) {
                leftShooter.setVelocity(leftOn ? targetVelocity : 0);
                middleShooter.setVelocity(middleOn ? targetVelocity : 0);
                rightShooter.setVelocity(rightOn ? targetVelocity : 0);
            } else {
                leftShooter.setVelocity(0);
                middleShooter.setVelocity(0);
                rightShooter.setVelocity(0);
            }
            prevTrigger = currentTrigger;

            // --- INTAKE CONTROL ---
            if (gamepad1.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // --- SERVO CONTROL ---
            servoPower = 0.0;
            if (gamepad1.a) {
                // Auto sorting logic during intake
                if (!isBallDetected(leftSensor)) {
                    servoPower = 1.0; // Clockwise (to the right, pushes to left)
                } else if (!isBallDetected(rightSensor)) {
                    servoPower = -1.0; // Counter-clockwise (to the left, pushes to right)
                } else {
                    servoPower = 0.0; // Stop to allow middle
                }
            } else {
                // Manual control with D-pad
                if (gamepad1.dpad_up) {
                    servoPower = 1.0;
                } else if (gamepad1.dpad_down) {
                    servoPower = -1.0;
                } else if (gamepad1.dpad_right) {
                    servoPower = 0.0;
                }
            }
            intakeServo.setPower(servoPower);

            // --- COLOR SENSOR DETECTION ---
            telemetry.addData("Left Ball", getBallColor(leftSensor));
            telemetry.addData("Middle Ball", getBallColor(middleSensor));
            telemetry.addData("Right Ball", getBallColor(rightSensor));

            // --- MISC CONTROLS ---
            if (gamepad1.x) {
                imuOffset = imu.getAngularOrientation().firstAngle;
            }

            if (gamepad1.right_bumper && slowToggleAllowed) {
                slowMode = !slowMode;
                slowToggleAllowed = false;
            }
            if (!gamepad1.right_bumper) {
                slowToggleAllowed = true;
            }

            telemetry.addData("Heading", "%.3f", heading);
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Centering", centering && tagVisible);
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("Shooters Running", shootersRunning);
            telemetry.update();
        }
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle - imuOffset;
    }

    private boolean isBallDetected(ColorSensor sensor) {
        return isGreen(sensor) || isPurple(sensor);
    }

    private String getBallColor(ColorSensor sensor) {
        if (isGreen(sensor)) {
            return "GREEN";
        } else if (isPurple(sensor)) {
            return "PURPLE";
        }
        return "NONE";
    }

    private boolean isGreen(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        return (green > red * 1.3 && green > blue * 1.3 && green > 80);
    }

    private boolean isPurple(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        // Tuned for darker purple: lower red/blue thresholds, adjusted ratios
        return (red > 40 && blue > 40 && green < red * 0.7 && green < blue * 0.7);
    }
}