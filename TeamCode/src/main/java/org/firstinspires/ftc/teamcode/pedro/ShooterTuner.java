
package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Tuner (Simple)", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    // ---------------- Hardware names ----------------
    private static final String LEFT_SHOOTER_NAME = "leftShooter";
    private static final String RIGHT_SHOOTER_NAME = "rightShooter";
    private static final String FEEDER_NAME = "feeder";
    private static final String INTAKE_NAME = "intake";

    // ---------------- Shooter PIDF ----------------
    private static final double SHOOTER_kP = 0.0005;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 0.00025;
    private static final double SHOOTER_I_MAX = 0.3;

    // ---------------- Shooter setup ----------------
    private static final double START_TARGET_RPM = 3000;
    private static final double BIG_STEP = 100;
    private static final double SMALL_STEP = 10;
    private static final double MAX_SHOOTER_RPM = 6000;
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final boolean LEFT_REVERSED = false;
    private static final boolean RIGHT_REVERSED = true;

    // ---------------- Feeder PID ----------------
    private static final double FEEDER_kP = 0.002;
    private static final double FEEDER_kI = 0.0;
    private static final double FEEDER_kD = 0.0;
    private static final double FEEDER_kF = 0.0004;
    private static final double FEEDER_I_MAX = 0.3;
    private static final double FEEDER_TARGET_RPM = 500;
    private static final double FEEDER_TICKS_PER_REV = 537.7;
    private static final boolean FEEDER_REVERSED = true;

    // ---------------- Options ----------------
    private static final boolean VOLTAGE_COMPENSATION = true;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // ---------------- Simple PIDF controller ----------------
    private static class PIDF {
        private double integral = 0;
        private double lastError = 0;
        private boolean first = true;

        void reset() {
            integral = 0;
            lastError = 0;
            first = true;
        }

        double update(double target, double measured,
                      double kP, double kI, double kD, double kF,
                      double iMax, double dt) {

            double error = target - measured;

            integral += error * dt;

            if (kI != 0) {
                double lim = iMax / Math.abs(kI);
                integral = Range.clip(integral, -lim, lim);
            } else {
                integral = 0;
            }

            double derivative = 0;

            if (!first && dt > 0) {
                derivative = (error - lastError) / dt;
            }

            first = false;
            lastError = error;

            return kP * error + kI * integral
                    + kD * derivative + kF * target;
        }
    }

    @Override
    public void runOpMode() {

        // ---------------- Initialize hardware ----------------
        DcMotorEx leftShooter = hardwareMap.get(
                DcMotorEx.class, LEFT_SHOOTER_NAME);

        DcMotorEx rightShooter = hardwareMap.get(
                DcMotorEx.class, RIGHT_SHOOTER_NAME);

        DcMotorEx feeder = hardwareMap.get(
                DcMotorEx.class, FEEDER_NAME);

        DcMotor intake = hardwareMap.get(
                DcMotor.class, INTAKE_NAME);

        // ---------------- Motor directions ----------------
        leftShooter.setDirection(
                LEFT_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);

        rightShooter.setDirection(
                RIGHT_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);

        feeder.setDirection(
                FEEDER_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);

        // Intake reversed
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Motor modes ----------------
        for (DcMotorEx m : new DcMotorEx[]{
                leftShooter, rightShooter, feeder}) {

            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        feeder.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);

        VoltageSensor battery =
                hardwareMap.voltageSensor.iterator().next();

        PIDF leftPIDF = new PIDF();
        PIDF rightPIDF = new PIDF();
        PIDF feederPIDF = new PIDF();

        double targetRPM = START_TARGET_RPM;

        boolean shooterOn = false;
        boolean feederOn = false;

        boolean lastX = false;
        boolean lastB = false;

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastLeft = false;
        boolean lastRight = false;

        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        loopTimer.reset();

        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();

            if (dt <= 0) dt = 1e-3;

            // ---------------- Gamepad controls ----------------

            // X toggles shooters
            if (gamepad1.x && !lastX) {
                shooterOn = !shooterOn;
            }

            // B toggles feeder
            if (gamepad1.b && !lastB) {
                feederOn = !feederOn;
            }

            // D-pad adjusts target RPM
            if (gamepad1.dpad_up && !lastUp) {
                targetRPM += BIG_STEP;
            }

            if (gamepad1.dpad_down && !lastDown) {
                targetRPM -= BIG_STEP;
            }

            if (gamepad1.dpad_right && !lastRight) {
                targetRPM += SMALL_STEP;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                targetRPM -= SMALL_STEP;
            }

            targetRPM = Range.clip(
                    targetRPM, 0, MAX_SHOOTER_RPM);

            lastX = gamepad1.x;
            lastB = gamepad1.b;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            // ---------------- Intake ----------------
            // Hold A = full power
            // Release A = stop
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // ---------------- Measure RPM ----------------
            double leftRPM =
                    leftShooter.getVelocity()
                            / SHOOTER_TICKS_PER_REV * 60.0;

            double rightRPM =
                    rightShooter.getVelocity()
                            / SHOOTER_TICKS_PER_REV * 60.0;

            double feederRPM =
                    feeder.getVelocity()
                            / FEEDER_TICKS_PER_REV * 60.0;

            // ---------------- Voltage compensation ----------------
            double voltageScale = 1.0;

            if (VOLTAGE_COMPENSATION) {
                double v = battery.getVoltage();

                if (v > 1) {
                    voltageScale = NOMINAL_VOLTAGE / v;
                }
            }

            // ---------------- Shooters ----------------
            double shooterTarget = shooterOn ? targetRPM : 0;

            double leftPower = 0;
            double rightPower = 0;

            if (shooterTarget > 0) {

                leftPower = leftPIDF.update(
                        shooterTarget,
                        leftRPM,
                        SHOOTER_kP,
                        SHOOTER_kI,
                        SHOOTER_kD,
                        SHOOTER_kF * voltageScale,
                        SHOOTER_I_MAX,
                        dt);

                rightPower = rightPIDF.update(
                        shooterTarget,
                        rightRPM,
                        SHOOTER_kP,
                        SHOOTER_kI,
                        SHOOTER_kD,
                        SHOOTER_kF * voltageScale,
                        SHOOTER_I_MAX,
                        dt);

                leftPower = Range.clip(leftPower, 0, 1);
                rightPower = Range.clip(rightPower, 0, 1);

            } else {
                leftPIDF.reset();
                rightPIDF.reset();
            }

            leftShooter.setPower(leftPower);
            rightShooter.setPower(rightPower);

            // ---------------- Feeder ----------------
            double feederTarget =
                    feederOn ? FEEDER_TARGET_RPM : 0;

            double feederPower = 0;

            if (feederTarget != 0) {

                feederPower = feederPIDF.update(
                        feederTarget,
                        feederRPM,
                        FEEDER_kP,
                        FEEDER_kI,
                        FEEDER_kD,
                        FEEDER_kF * voltageScale,
                        FEEDER_I_MAX,
                        dt);

                feederPower = Range.clip(
                        feederPower, -1, 1);

            } else {
                feederPIDF.reset();
            }

            feeder.setPower(feederPower);

            // ---------------- Telemetry ----------------
            telemetry.addData(
                    "Shooter", shooterOn ? "ON" : "OFF");

            telemetry.addData(
                    "Target RPM", "%.0f", targetRPM);

            telemetry.addData(
                    "Left RPM", "%.0f (err %.0f)",
                    leftRPM, shooterTarget - leftRPM);

            telemetry.addData(
                    "Right RPM", "%.0f (err %.0f)",
                    rightRPM, shooterTarget - rightRPM);

            telemetry.addData(
                    "Left / Right power", "%.2f / %.2f",
                    leftPower, rightPower);

            telemetry.addLine();

            telemetry.addData(
                    "Feeder", feederOn ? "ON" : "OFF");

            telemetry.addData(
                    "Feeder RPM", "%.0f / %.0f",
                    feederRPM, feederTarget);

            telemetry.addData(
                    "Feeder power", "%.2f", feederPower);

            telemetry.addLine();

            telemetry.addData(
                    "Intake", gamepad1.a ? "ON" : "OFF");

            telemetry.addData(
                    "Intake power", "%.1f",
                    intake.getPower());

            telemetry.addLine();

            telemetry.addData(
                    "Battery V", "%.2f",
                    battery.getVoltage());

            telemetry.update();
        }

        // ---------------- Stop all motors ----------------
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        feeder.setPower(0);
        intake.setPower(0);
    }
}
