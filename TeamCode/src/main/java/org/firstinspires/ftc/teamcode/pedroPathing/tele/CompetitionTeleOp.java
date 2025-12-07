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
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Configurable
@TeleOp(name = "Testing TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    // -----------------------------------------------------------------------
    // Exposed (public static) shooter PIDF + target RPM values for tuning in Panels
    // -----------------------------------------------------------------------
    // Right shooter (default values)
    public static double right_kP = 0.0002;
    public static double right_kI = 0.0;
    public static double right_kD = 0.0001;
    public static double right_kF = 0.00025;
    public static double right_targetRPM = 3500.0;

    // Middle shooter
    public static double middle_kP = 0.00015;
    public static double middle_kI = 0.0;
    public static double middle_kD = 0.0001;
    public static double middle_kF = 0.0002;
    public static double middle_targetRPM = 3500.0;

    // Left shooter
    public static double left_kP = 0.00015;
    public static double left_kI = 0.0;
    public static double left_kD = 0.0001;
    public static double left_kF = 0.0002;
    public static double left_targetRPM = 3500.0;

    // -----------------------------------------------------------------------
    // Exposed Limelight PID values for tuning in Panels
    // -----------------------------------------------------------------------
    public static double limelight_kP = 0.012;
    public static double limelight_kD = 0.002;
    public static double limelight_minCmd = 0.06;
    public static double limelight_deadband = 1.8;
    public static double limelight_maxSpeed = 0.3;

    // -----------------------------------------------------------------------
    // Shooter RPM tolerance and vibration settings
    // -----------------------------------------------------------------------
    public static double rpm_tolerance = 100.0; // RPM within tolerance to trigger vibration
    public static int vibration_duration_ms = 200; // Duration of vibration in milliseconds

    // -----------------------------------------------------------------------
    // Other TeleOp / limelight constants

    // -----------------------------------------------------------------------
    private static final double TRIGGER_THRESHOLD = 0.3;

    // -----------------------------------------------------------------------
    // Hardware & helpers
    // -----------------------------------------------------------------------
    private Follower follower;
    private TelemetryManager telemetryM;

    private BNO055IMU imu;
    private double imuOffset = 0;

    private Limelight3A limelight;

    // servos (triple shooter)
    private Servo leftServo;
    private Servo middleServo;
    private Servo rightServo;
    private Servo leftPushServo;
    private Servo rightPushServo;
    private static final double SERVO_MAX = 1.0; // startup / up
    private static final double SERVO_MIN = 0.0; // down during centering

    // intake
    private DcMotor intake;

    // sorter (continuous rotation) and color sensors (left/right)
    private CRServo sorterServo;
    private CRServo sorterServo2;
    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;

    // thresholds (changed to 100)
    private static final int SIDE_DETECT_THRESHOLD = 100;   // left/right sensors

    // shooter motors (use DcMotorEx for getVelocity())
    private DcMotorEx rightShooter;
    private DcMotorEx middleShooter;
    private DcMotorEx leftShooter;

    // PID integrators / state for each shooter
    private double right_integral = 0.0;
    private double right_lastError = 0.0;

    private double middle_integral = 0.0;
    private double middle_lastError = 0.0;

    private double left_integral = 0.0;
    private double left_lastError = 0.0;

    // ---- shooter ENABLE flags (D-pad toggles) ----
    private boolean rightShooterEnabled  = false;
    private boolean middleShooterEnabled = false;
    private boolean leftShooterEnabled   = false;

    private boolean dpadLeftReady  = true;
    private boolean dpadRightReady = true;
    private boolean dpadUpReady    = true;

    // ---- slow mode toggle ----
    private boolean slowModeEnabled = false;
    private boolean rightBumperReady = true;

    // ---- sorter servo toggle and state ----
    private boolean sorterEnabled = true;
    private boolean rightStickReady = true;

    // ---- push servo delay tracking ----
    private boolean triggerWasPressed = false;
    private double triggerPressTime = 0.0;
    private static final double PUSH_SERVO_DELAY = 0.25; // half second delay

    // ---- vibration state tracking ----
    private boolean wasAtTargetRPM = false;

    // encoder ticks per motor revolution for REV (change if using gearbox encoder)
    private static final double TICKS_PER_REV = 28.0;

    // rotation PID memory
    private double lastTxError = 0.0;
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);

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
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- SORTER + COLOR SENSORS ----------
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        sorterServo2 = hardwareMap.get(CRServo.class, "sorterServo2");
        leftColor   = hardwareMap.get(RevColorSensorV3.class, "leftColor");
        rightColor  = hardwareMap.get(RevColorSensorV3.class, "rightColor");

        // ---------- SERVOS ----------
        leftServo   = hardwareMap.get(Servo.class, "leftServo");
        middleServo = hardwareMap.get(Servo.class, "middleServo");
        rightServo  = hardwareMap.get(Servo.class, "rightServo");
        leftPushServo = hardwareMap.get(Servo.class, "leftPushServo");
        rightPushServo  = hardwareMap.get(Servo.class, "rightPushServo");

        // Reverse the rightPushServo direction
        rightPushServo.setDirection(Servo.Direction.REVERSE);

        // startup positions
        leftServo.setPosition(SERVO_MAX);
        middleServo.setPosition(SERVO_MAX);
        rightServo.setPosition(SERVO_MAX);
        leftPushServo.setPosition(SERVO_MAX);
        rightPushServo.setPosition(SERVO_MAX);  // Now uses same value as others due to direction reverse


        // ---------- SHOOTER MOTORS (DcMotorEx) ----------
        rightShooter  = hardwareMap.get(DcMotorEx.class, "rightShooter");
        middleShooter = hardwareMap.get(DcMotorEx.class, "middleShooter");
        leftShooter   = hardwareMap.get(DcMotorEx.class, "leftShooter");

        // Use encoder velocity reporting
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // direction and zero behavior - adjust if necessary
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        middleShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ---------- READY ----------
        telemetry.addLine("TeleOp Triple Shooter Ready");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive(true);

        lastLoopTime = getRuntime();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.update();

            // loop timing
            double now = getRuntime();
            double dt = now - lastLoopTime;
            if (dt <= 0) dt = 1e-6;
            lastLoopTime = now;

            // ---------- SORTER SERVOS: Default spinning with manual override ----------
            // Right stick click toggles sorter on/off
            if (gamepad2.right_stick_button && rightStickReady) {
                sorterEnabled = !sorterEnabled;
                rightStickReady = false;
            }
            if (!gamepad2.right_stick_button) {
                rightStickReady = true;
            }

            // Determine sorter power based on state and manual override
            double sorterPower1 = 0.0;
            double sorterPower2 = 0.0;

            if (sorterEnabled) {
                // Check for manual override with left stick
                double leftStickX = gamepad2.left_stick_x;
                if (Math.abs(leftStickX) > 0.1) { // deadband
                    // Manual override: left = counter-clockwise, right = clockwise
                    sorterPower1 = leftStickX;   //Was originally -leftStickX
                    sorterPower2 = leftStickX;
                } else {
                    // Default: both spinning in opposite directions
                    sorterPower1 = 1.0;
                    sorterPower2 = -1.0;
                }
            }
            // If sorterEnabled is false, both powers stay at 0.0

            sorterServo.setPower(sorterPower1);
            sorterServo2.setPower(sorterPower2);

            // ---------- LIMELIGHT READ ----------
            LLResult result = limelight.getLatestResult();
            boolean tagVisible = false;
            int tagId = -1;
            double tx = 0.0;
            if (result != null && result.isValid()) {
                List<FiducialResult> fid = result.getFiducialResults();
                if (!fid.isEmpty()) {
                    tagVisible = true;
                    FiducialResult primary = fid.get(0);
                    tagId = primary.getFiducialId();
                    tx = primary.getTargetXDegrees();
                }
            }

            // ---------- SLOW MODE TOGGLE ----------
            if (gamepad1.right_bumper && rightBumperReady) {
                slowModeEnabled = !slowModeEnabled;
                rightBumperReady = false;
            }
            if (!gamepad1.right_bumper) {
                rightBumperReady = true;
            }

            // ---------- DRIVER INPUTS ----------
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double heading = getHeading();

            // Apply cubic inputs
            y = Math.pow(y, 3);
            x = Math.pow(x, 3);
            rx = Math.pow(rx, 3);

            // Apply slow mode scaling (max power 0.25 when enabled, 1.0 when disabled)
            double maxPower = slowModeEnabled ? 0.25 : 1.0;

            double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

            boolean centering = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean centeringGamepad2 = gamepad2.right_trigger > TRIGGER_THRESHOLD;

            // ---------- SERVOS + INTAKE behavior linked to RIGHT TRIGGER ----------
            // Only gamepad2 right trigger controls servos and intake
            if (centeringGamepad2) {
                // Track when trigger is first pressed
                if (!triggerWasPressed) {
                    triggerPressTime = now;
                    triggerWasPressed = true;
                }

                // Calculate time since trigger was pressed
                double timeSinceTriggerPress = now - triggerPressTime;

                // Blocker servos go down immediately
                leftServo.setPosition(SERVO_MIN);
                middleServo.setPosition(SERVO_MIN);
                rightServo.setPosition(SERVO_MIN);

                // Push servos go down after delay
                if (timeSinceTriggerPress >= PUSH_SERVO_DELAY) {
                    leftPushServo.setPosition(SERVO_MIN);
                    rightPushServo.setPosition(SERVO_MIN);
                } else {
                    // Keep push servos up during delay
                    leftPushServo.setPosition(SERVO_MAX);
                    rightPushServo.setPosition(SERVO_MAX);
                }

                // intake forced forward while trigger pressed
                //intake.setPower(1.0);
            } else {
                // Reset trigger tracking when released
                triggerWasPressed = false;

                // All servos up
                leftServo.setPosition(SERVO_MAX);
                middleServo.setPosition(SERVO_MAX);
                rightServo.setPosition(SERVO_MAX);
                leftPushServo.setPosition(SERVO_MAX);
                rightPushServo.setPosition(SERVO_MAX);

                // manual intake control when trigger not pressed (from gamepad2)
                if (gamepad2.a) intake.setPower(1.0);
                else if (gamepad2.b) intake.setPower(-1.0);
                else intake.setPower(0.0);
            }

            // ---------- D-PAD SHOOTER TOGGLES (gamepad2) ----------
            // Left trigger on gamepad2 turns off all shooters immediately
            if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                leftShooterEnabled = false;
                middleShooterEnabled = false;
                rightShooterEnabled = false;
            }

            // dpad left -> left shooter, dpad right -> right shooter, dpad up -> middle shooter
            if (gamepad2.dpad_left && dpadLeftReady) {
                leftShooterEnabled = !leftShooterEnabled;
                dpadLeftReady = false;
            }
            if (!gamepad2.dpad_left) dpadLeftReady = true;

            if (gamepad2.dpad_right && dpadRightReady) {
                rightShooterEnabled = !rightShooterEnabled;
                dpadRightReady = false;
            }
            if (!gamepad2.dpad_right) dpadRightReady = true;

            if (gamepad2.dpad_up && dpadUpReady) {
                middleShooterEnabled = !middleShooterEnabled;
                dpadUpReady = false;
            }
            if (!gamepad2.dpad_up) dpadUpReady = true;

            // ---------- Shooter PIDF loops (independent) ----------
            double rightPower = 0.0, middlePower = 0.0, leftPower = 0.0;
            double rightRPM = 0.0, middleRPM = 0.0, leftRPM = 0.0;

            // get velocities (ticks/sec)
            double rightTicksPerSec  = rightShooter.getVelocity();
            double middleTicksPerSec = middleShooter.getVelocity();
            double leftTicksPerSec   = leftShooter.getVelocity();

            rightRPM  = rightTicksPerSec  * 60.0 / TICKS_PER_REV;
            middleRPM = middleTicksPerSec * 60.0 / TICKS_PER_REV;
            leftRPM   = leftTicksPerSec   * 60.0 / TICKS_PER_REV;

            // RIGHT shooter PIDF
            if (rightShooterEnabled) {
                double error = right_targetRPM - rightRPM;
                right_integral += error * dt;
                double derivative = (error - right_lastError) / dt;
                right_lastError = error;

                double p = right_kP * error;
                double i = right_kI * right_integral;
                double d = right_kD * derivative;
                double f = right_kF * right_targetRPM;
                rightPower = p + i + d + f;
                rightPower = Range.clip(rightPower, 0.0, 1.0);
            } else {
                rightPower = 0.0;
            }

            // MIDDLE shooter PIDF
            if (middleShooterEnabled) {
                double error = middle_targetRPM - middleRPM;
                middle_integral += error * dt;
                double derivative = (error - middle_lastError) / dt;
                middle_lastError = error;

                double p = middle_kP * error;
                double i = middle_kI * middle_integral;
                double d = middle_kD * derivative;
                double f = middle_kF * middle_targetRPM;
                middlePower = p + i + d + f;
                middlePower = Range.clip(middlePower, 0.0, 1.0);
            } else {
                middlePower = 0.0;
            }

            // LEFT shooter PIDF
            if (leftShooterEnabled) {
                double error = left_targetRPM - leftRPM;
                left_integral += error * dt;
                double derivative = (error - left_lastError) / dt;
                left_lastError = error;

                double p = left_kP * error;
                double i = left_kI * left_integral;
                double d = left_kD * derivative;
                double f = left_kF * left_targetRPM;
                leftPower = p + i + d + f;
                leftPower = Range.clip(leftPower, 0.0, 1.0);
            } else {
                leftPower = 0.0;
            }

            // apply shooter powers
            rightShooter.setPower(rightPower);
            middleShooter.setPower(middlePower);
            leftShooter.setPower(leftPower);

            // ---------- CHECK IF SHOOTERS ARE AT TARGET RPM ----------
            boolean allEnabledShootersAtTarget = false;
            boolean anyShooterEnabled = leftShooterEnabled || middleShooterEnabled || rightShooterEnabled;

            if (anyShooterEnabled) {
                boolean allAtTarget = true;

                // Check each enabled shooter
                if (leftShooterEnabled) {
                    allAtTarget &= Math.abs(leftRPM - left_targetRPM) <= rpm_tolerance;
                }
                if (middleShooterEnabled) {
                    allAtTarget &= Math.abs(middleRPM - middle_targetRPM) <= rpm_tolerance;
                }
                if (rightShooterEnabled) {
                    allAtTarget &= Math.abs(rightRPM - right_targetRPM) <= rpm_tolerance;
                }

                allEnabledShootersAtTarget = allAtTarget;
            }

            // Vibrate gamepad2 when shooters reach target (edge trigger - only on transition)
            if (allEnabledShootersAtTarget && !wasAtTargetRPM && anyShooterEnabled) {
                gamepad2.rumble(vibration_duration_ms);
            }
            wasAtTargetRPM = allEnabledShootersAtTarget && anyShooterEnabled;

            // ---------- COLOR SENSORS: read values & occupancy ----------
            int lR = leftColor.red();
            int lG = leftColor.green();
            int lB = leftColor.blue();

            int rR = rightColor.red();
            int rG = rightColor.green();
            int rB = rightColor.blue();

            boolean leftOccupied  = (lR > SIDE_DETECT_THRESHOLD) || (lG > SIDE_DETECT_THRESHOLD) || (lB > SIDE_DETECT_THRESHOLD);
            boolean rightOccupied = (rR > SIDE_DETECT_THRESHOLD) || (rG > SIDE_DETECT_THRESHOLD) || (rB > SIDE_DETECT_THRESHOLD);

            // ---------- Driving / auto-rotation (centering) ----------
            // Only gamepad1 trigger can activate auto-centering
            if (centering && tagVisible) {
                double rotate = 0.0;
                double error = tx;
                if (Math.abs(error) > limelight_deadband) {
                    double derivative = (error - lastTxError) / dt;
                    rotate = -(limelight_kP * error + limelight_kD * derivative);

                    if (Math.abs(rotate) < limelight_minCmd) {
                        rotate = limelight_minCmd * Math.signum(rotate);
                    }
                    rotate = Range.clip(rotate, -limelight_maxSpeed, limelight_maxSpeed);
                } else {
                    rotate = 0.0;
                }
                lastTxError = error;
                follower.setTeleOpDrive(0.0, 0.0, rotate, false);
            } else {
                follower.setTeleOpDrive(rotatedY * maxPower, rotatedX * maxPower, rx * maxPower, true);
            }

            // ---------- Reset heading (X) ----------
            if (gamepad1.x) imuOffset = imu.getAngularOrientation().firstAngle;

            // ---------- Telemetry ----------
            telemetry.addData("LeftColor R/G/B", "%d / %d / %d", lR, lG, lB);
            telemetry.addData("RightColor R/G/B", "%d / %d / %d", rR, rG, rB);

            telemetry.addData("LeftOccupied", leftOccupied);
            telemetry.addData("RightOccupied", rightOccupied);

            telemetry.addLine();
            telemetry.addData("Slow Mode", slowModeEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Max Power", maxPower);

            telemetry.addLine();
            telemetry.addData("Shooter Enabled (L/M/R)", "%b / %b / %b", leftShooterEnabled, middleShooterEnabled, rightShooterEnabled);
            telemetry.addData("Shooter RPMs (L/M/R)", "%.0f / %.0f / %.0f", leftRPM, middleRPM, rightRPM);
            telemetry.addData("Shooter Pwr  (L/M/R)", "%.3f / %.3f / %.3f", leftPower, middlePower, rightPower);
            telemetry.addData("At Target RPM", allEnabledShootersAtTarget && anyShooterEnabled ? "YES" : "NO");

            telemetry.addLine();
            telemetry.addData("Sort Enabled", sorterEnabled);
            telemetry.addData("Sort Power", sorterServo.getPower());
            telemetry.addData("Sort2 Power", sorterServo2.getPower());
            telemetry.addData("Intake Power", intake.getPower());

            if (tagVisible) {
                telemetry.addData("Tag ID", tagId);
                telemetry.addData("tx (deg)", "%.2f", tx);
            } else {
                telemetry.addLine("No AprilTag");
            }

            telemetry.update();
        } // end opModeIsActive
    } // end runOpMode

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle - imuOffset;
    }
}