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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;


@Configurable
@TeleOp(name = "Limelight + Intake + Servo + Color Sensor (Updated)", group = "TeleOp")
public class TeleControl extends LinearOpMode {


    private Follower follower;
    private TelemetryManager telemetryM;
    private BNO055IMU imu;
    private double imuOffset = 0;
    private Limelight3A limelight;


    // === HARDWARE ===
    private DcMotor intakeMotor;
    private CRServo intakeServo;                // renamed from motor67 to intakeServo
    private ColorSensor leftSensor;            // left lane color sensor
    private ColorSensor rightSensor;           // right lane color sensor
    private ColorSensor middleSensor;          // middle lane color sensor

    // Shooter motors (3) — each uses its motor encoder
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor shooter3;


    // === CONSTANTS ===
    private static final double ROTATE_KP = 0.035;
    private static final double TX_DEADBAND = 1.5;
    private static final double MAX_ROTATE_SPEED = 0.6;
    private static final double TRIGGER_THRESHOLD = 0.3;

    // Encoder/PID constants for shooters — tune these for your hardware
    private static final double TARGET_RPM = 3000.0;
    private static final int TICKS_PER_REV = 28; // <-- SET THIS to your motor's encoder ticks per revolution
    private static final double SHOOTER_KP = 0.0005; // very small — tune
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.0;


    private boolean slowMode = false;
    private boolean slowToggleAllowed = true;
    private double slowModeMultiplier = 0.25;   //Quarter speed

    // --- shooter state ---
    private boolean shootersActive = false;          // true when shooter PIDs are running
    private boolean shootersRequested = false;       // rising-edge detection for right trigger
    private boolean shooterLeftInitially = false;   // which lanes had balls when shooters were started
    private boolean shooterMiddleInitially = false;
    private boolean shooterRightInitially = false;

    // For RPM measurement
    private int lastEnc1 = 0, lastEnc2 = 0, lastEnc3 = 0;
    private long lastEncTimeMs = 0;

    // simple integrators / PID states
    private double integrator1 = 0, integrator2 = 0, integrator3 = 0;
    private double lastError1 = 0, lastError2 = 0, lastError3 = 0;


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


        // ---------- SERVO (renamed) ----------
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");


        // ---------- COLOR SENSORS (3) ----------
        leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
        middleSensor = hardwareMap.get(ColorSensor.class, "middleSensor");


        // ---------- SHOOTERS ----------
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotor.class, "shooter3");

        // use encoders for feedback
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addLine("All systems ready: Limelight + Intake + Servo + Color Sensors + Shooters");
        telemetry.update();


        waitForStart();
        follower.startTeleopDrive(true);


        double servoPower = 0.0;

        // initialize enc counts/time for RPM calc
        lastEnc1 = shooter1.getCurrentPosition();
        lastEnc2 = shooter2.getCurrentPosition();
        lastEnc3 = shooter3.getCurrentPosition();
        lastEncTimeMs = System.currentTimeMillis();

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


            boolean centering = gamepad1.right_trigger > TRIGGER_THRESHOLD;


            if (centering && tagVisible) {
                double rotate = 0.0;
                if (Math.abs(tx) > TX_DEADBAND) {
                    // small-tuned centering control — use a deadband and proportional control
                    rotate = -tx * ROTATE_KP;
                    rotate = com.qualcomm.robotcore.util.Range.clip(rotate, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);
                }
                follower.setTeleOpDrive(0, 0, rotate, false);
            } else {
                follower.setTeleOpDrive(rotatedY * scale, rotatedX * scale, rx * scale, true);
            }


            // --- INTAKE CONTROL ---
            if (gamepad1.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }


            // --- SERVO CONTROL (D-pad remains functional) ---
            if (gamepad1.dpad_up) {
                // manual override via dpad
                servoPower = 1.0;
            } else if (gamepad1.dpad_down) {
                servoPower = -1.0;
            } else if (gamepad1.dpad_right) {
                servoPower = 0.0;
            }

            // If intake is running inwards (gamepad1.a) then perform sorting logic and override manual dpad unless dpad explicitly set
            if (gamepad1.a) {
                // decide which lanes have balls right now
                boolean leftHas = detectBall(leftSensor);
                boolean rightHas = detectBall(rightSensor);
                boolean middleHas = detectBall(middleSensor);

                // telemetry of lane occupancy
                telemetry.addData("LaneHas (L,M,R)", "%b, %b, %b", leftHas, middleHas, rightHas);

                // Sorting rules (priority):
                // 1) If left is EMPTY -> spin clockwise (to the right) until leftSensor detects a ball
                // 2) else if right is EMPTY -> spin counter-clockwise until rightSensor detects a ball
                // 3) else if middle is EMPTY -> stop to let the ball go through
                // 4) else (all full) -> stop

                if (!leftHas) {
                    // spin clockwise until leftSensor sees a ball
                    servoPower = 1.0; // clockwise / to the right
                } else if (!rightHas) {
                    // left full, right empty: spin CCW to feed right lane
                    servoPower = -1.0;
                } else if (!middleHas) {
                    // let the ball go down the middle
                    servoPower = 0.0;
                } else {
                    // all full -> stop
                    servoPower = 0.0;
                }

                // additionally, watch for sensor transitions and print when a sensor sees a ball
                // (we'll print if sensor currently sees a ball)
                if (leftHas) telemetry.addLine("LeftSensor: BALL detected (GREEN or PURPLE)");
                if (rightHas) telemetry.addLine("RightSensor: BALL detected (GREEN or PURPLE)");
                if (middleHas) telemetry.addLine("MiddleSensor: BALL detected (GREEN or PURPLE)");

            }

            // apply servo power
            intakeServo.setPower(servoPower);


            // --- COLOR SENSOR DETECTION (legacy single sensor info removed) ---
            // individual sensors are used above; also show RGB values for middle sensor for debug
            int mR = middleSensor.red();
            int mG = middleSensor.green();
            int mB = middleSensor.blue();

            boolean midGreen = isGreen(mR, mG, mB);
            boolean midPurple = isPurple(mR, mG, mB);

            if (midGreen) telemetry.addLine("Middle: GREEN detected");
            else if (midPurple) telemetry.addLine("Middle: PURPLE detected");


            // --- SHOOTER CONTROL ---
            // Rising edge detection for right trigger press while centering on AprilTag
            boolean triggerPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            if (triggerPressed && !shootersRequested && tagVisible) {
                // start/stop shooters (toggle)
                shootersActive = !shootersActive;
                if (shootersActive) {
                    // when starting, record which lanes currently have balls — those shooters will run
                    shooterLeftInitially = detectBall(leftSensor);
                    shooterMiddleInitially = detectBall(middleSensor);
                    shooterRightInitially = detectBall(rightSensor);

                    telemetry.addLine("Starting shooters for lanes (L,M,R): " + shooterLeftInitially + ", " + shooterMiddleInitially + ", " + shooterRightInitially);
                } else {
                    // stopping: zero motors and reset PID integrators
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                    shooter3.setPower(0);
                    integrator1 = integrator2 = integrator3 = 0;
                    lastError1 = lastError2 = lastError3 = 0;
                    telemetry.addLine("Stopping shooters");
                }
            }
            shootersRequested = triggerPressed;


            // If shootersActive, run PID loops for motors that were initially requested
            long nowMs = System.currentTimeMillis();
            double dt = Math.max(1, nowMs - lastEncTimeMs) / 1000.0;

            int curEnc1 = shooter1.getCurrentPosition();
            int curEnc2 = shooter2.getCurrentPosition();
            int curEnc3 = shooter3.getCurrentPosition();

            double rpm1 = 0, rpm2 = 0, rpm3 = 0;
            if (dt > 0) {
                rpm1 = (curEnc1 - lastEnc1) / (double) TICKS_PER_REV * (60.0 / dt);
                rpm2 = (curEnc2 - lastEnc2) / (double) TICKS_PER_REV * (60.0 / dt);
                rpm3 = (curEnc3 - lastEnc3) / (double) TICKS_PER_REV * (60.0 / dt);
            }

            lastEnc1 = curEnc1;
            lastEnc2 = curEnc2;
            lastEnc3 = curEnc3;
            lastEncTimeMs = nowMs;


            if (shootersActive) {
                // shooter1
                if (shooterLeftInitially) {
                    double out1 = pidCompute(TARGET_RPM, rpm1, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, dt, 1);
                    shooter1.setPower(out1);
                }
                // shooter2
                if (shooterMiddleInitially) {
                    double out2 = pidCompute(TARGET_RPM, rpm2, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, dt, 2);
                    shooter2.setPower(out2);
                }
                // shooter3
                if (shooterRightInitially) {
                    double out3 = pidCompute(TARGET_RPM, rpm3, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, dt, 3);
                    shooter3.setPower(out3);
                }
            }


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
            telemetry.addData("Middle RGB", "%d, %d, %d", mR, mG, mB);
            telemetry.addData("Middle RPMs", "%.1f", rpm2);
            telemetry.addData("Shooters Active", shootersActive);
            telemetry.update();
        }
    }


    private double getHeading() {
        return imu.getAngularOrientation().firstAngle - imuOffset;
    }


    // Detect whether a sensor currently sees a ball (green or purple). Tuned to detect darker purple.
    private boolean detectBall(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        return isGreen(r, g, b) || isPurple(r, g, b);
    }

    private boolean isGreen(int r, int g, int b) {
        // maintain original good green detection but relax thresholds slightly for noisy conditions
        return (g > r * 1.25 && g > b * 1.25 && g > 70);
    }

    private boolean isPurple(int r, int g, int b) {
        // tuned to detect darker purple hues: purple often has both red and blue present while green is low
        // Accept lower absolute values and use ratios — this helps detect darker (low-intensity) purples
        boolean ratioCondition = (r > 20 && b > 20) && (g < (r + b) * 0.45);
        boolean absCondition = (r > 30 || b > 30); // at least one channel somewhat present
        return ratioCondition && absCondition;
    }

    // Very simple PID controller per motor — maintains integrators and errors by motor id (1-3)
    private double pidCompute(double targetRpm, double currentRpm, double kP, double kI, double kD, double dt, int motorId) {
        double error = targetRpm - currentRpm;
        double output = 0;
        if (motorId == 1) {
            integrator1 += error * dt;
            double derivative = (error - lastError1) / dt;
            output = kP * error + kI * integrator1 + kD * derivative;
            lastError1 = error;
        } else if (motorId == 2) {
            integrator2 += error * dt;
            double derivative = (error - lastError2) / dt;
            output = kP * error + kI * integrator2 + kD * derivative;
            lastError2 = error;
        } else {
            integrator3 += error * dt;
            double derivative = (error - lastError3) / dt;
            output = kP * error + kI * integrator3 + kD * derivative;
            lastError3 = error;
        }

        // clip output to motor power range
        output = com.qualcomm.robotcore.util.Range.clip(output, -1.0, 1.0);
        return output;
    }
}
