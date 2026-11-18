package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Feeder Test", group = "Test")
public class Shooter_Feeder_test extends LinearOpMode {

    // ================= EDIT THESE =================

    // --- Starting directions (you can also flip them live, see controls) ---
    private static final boolean LEFT_SHOOTER_REVERSED  = true;
    private static final boolean RIGHT_SHOOTER_REVERSED = false;
    private static final boolean FEEDER_REVERSED        = false;

    // --- Shooter target ---
    private static double targetRPM = 3250;            // FLYWHEEL rpm
    private static final double RPM_STEP = 50;         // D-pad up/down step
    private static final double RPM_TOLERANCE = 100;   // "READY" window (+/- RPM)

    // --- REV HD Hex Motor encoder ---
    // The encoder is on the motor shaft: 28 counts per MOTOR revolution.
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_FREE_SPEED_RPM = 6000;   // bare HD Hex motor

    // Motor-to-flywheel reduction. 1.0 = no gearbox / direct 1:1.
    // With UltraPlanetary cartridges, multiply the ACTUAL ratios
    // (3:1 ~ 2.89, 4:1 ~ 3.61, 5:1 ~ 5.23). Use a value < 1.0 if you gear UP (belt/gears).
    private static final double SHOOTER_GEAR_RATIO = 1.0;

    // --- Shooter PIDF ---
    // Encoder ticks/sec at motor free speed (independent of gearbox since the encoder is on the motor)
    private static final double MAX_TICKS_PER_SEC = MOTOR_FREE_SPEED_RPM * MOTOR_TICKS_PER_REV / 60.0; // 2800
    private static double kF = 50; // ~11.7
    private static double kP = 0.1;                    // ~1.17
    private static double kI = 0.1;                    // ~0.117
    private static double kD = 0.0;

    // --- Feeder ---
    private static final double FEEDER_POWER = 0.70;

    // ==============================================

    private DcMotorEx leftShooter, rightShooter, feeder;   // null if not found/plugged in

    private boolean leftRev  = LEFT_SHOOTER_REVERSED;
    private boolean rightRev = RIGHT_SHOOTER_REVERSED;
    private boolean feederRev = FEEDER_REVERSED;

    private boolean shooterOn = false;
    private boolean lastX, lastY, lastLB, lastRB, lastUp, lastDown;

    @Override
    public void runOpMode() {
        // tryGet returns null instead of crashing, so you can add motors one at a time
        leftShooter  = hardwareMap.tryGet(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.tryGet(DcMotorEx.class, "rightShooter");
        feeder       = hardwareMap.tryGet(DcMotorEx.class, "Feeder");

        for (DcMotorEx m : new DcMotorEx[]{leftShooter, rightShooter}) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // required for velocity PIDF
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        if (feeder != null) {
            feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        applyDirections();

        // Direction toggles work during INIT too, so you can fix them before pressing start
        while (!isStarted() && !isStopRequested()) {
            handleDirectionToggles();
            addControlsTelemetry();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            handleDirectionToggles();

            // ---- Toggle shooter ----
            boolean rb = gamepad1.right_bumper;
            if (rb && !lastRB) shooterOn = !shooterOn;
            lastRB = rb;

            // ---- Adjust target RPM live ----
            if (gamepad1.dpad_up && !lastUp)     targetRPM += RPM_STEP;
            if (gamepad1.dpad_down && !lastDown) targetRPM = Math.max(0, targetRPM - RPM_STEP);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // ---- Shooter velocity control ----
            double ticksPerFlywheelRev = MOTOR_TICKS_PER_REV * SHOOTER_GEAR_RATIO;
            double targetTicksPerSec = shooterOn ? targetRPM * ticksPerFlywheelRev / 60.0 : 0;
            if (leftShooter != null)  leftShooter.setVelocity(targetTicksPerSec);
            if (rightShooter != null) rightShooter.setVelocity(targetTicksPerSec);

            // ---- Feeder ----
            if (feeder != null) {
                if (gamepad1.a)      feeder.setPower(FEEDER_POWER);
                else if (gamepad1.b) feeder.setPower(-FEEDER_POWER);
                else                 feeder.setPower(0);
            }

            // ---- Measured RPM from each motor's own encoder ----
            double leftRPM  = leftShooter  == null ? 0 : Math.abs(leftShooter.getVelocity())  * 60.0 / ticksPerFlywheelRev;
            double rightRPM = rightShooter == null ? 0 : Math.abs(rightShooter.getVelocity()) * 60.0 / ticksPerFlywheelRev;

            boolean ready = shooterOn && (leftShooter != null || rightShooter != null);
            if (leftShooter != null)  ready &= Math.abs(leftRPM - targetRPM)  < RPM_TOLERANCE;
            if (rightShooter != null) ready &= Math.abs(rightRPM - targetRPM) < RPM_TOLERANCE;

            addControlsTelemetry();
            telemetry.addLine();
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Left RPM", leftShooter == null ? "not found" : String.format("%.0f", leftRPM));
            telemetry.addData("Right RPM", rightShooter == null ? "not found" : String.format("%.0f", rightRPM));
            telemetry.addData("READY TO FEED", ready ? "YES" : "no");
            telemetry.addData("Feeder power", feeder == null ? "not found" : String.format("%.2f", feeder.getPower()));

            double maxFlywheelRPM = MOTOR_FREE_SPEED_RPM / SHOOTER_GEAR_RATIO;
            if (targetRPM > 0.85 * maxFlywheelRPM) {
                telemetry.addLine("WARNING: target is above ~85% of max flywheel RPM ("
                        + (int) maxFlywheelRPM + "). May not be reachable.");
            }

            DcMotorEx pidfMotor = leftShooter != null ? leftShooter : rightShooter;
            if (pidfMotor != null) {
                PIDFCoefficients pidf = pidfMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("PIDF", "P=%.2f I=%.2f D=%.2f F=%.2f", pidf.p, pidf.i, pidf.d, pidf.f);
            }
            telemetry.update();
        }

        if (leftShooter != null)  leftShooter.setVelocity(0);
        if (rightShooter != null) rightShooter.setVelocity(0);
        if (feeder != null)       feeder.setPower(0);
    }

    /** X = leftShooter, Y = rightShooter, Left Bumper = Feeder. Edge-detected. */
    private void handleDirectionToggles() {
        boolean x = gamepad1.x, y = gamepad1.y, lb = gamepad1.left_bumper;

        // Shooter flips are ignored while the shooter is spinning (prevents violent reversals)
        if (!shooterOn) {
            if (x && !lastX) { leftRev  = !leftRev;  applyDirections(); }
            if (y && !lastY) { rightRev = !rightRev; applyDirections(); }
        }
        if (lb && !lastLB) { feederRev = !feederRev; applyDirections(); }

        lastX = x; lastY = y; lastLB = lb;
    }

    private void applyDirections() {
        if (leftShooter != null)
            leftShooter.setDirection(leftRev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (rightShooter != null)
            rightShooter.setDirection(rightRev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (feeder != null)
            feeder.setDirection(feederRev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void addControlsTelemetry() {
        telemetry.addLine("Gamepad1: RB=shooter on/off | A/B=feeder fwd/rev | DPad Up/Down=RPM");
        telemetry.addLine("Flip direction: X=leftShooter | Y=rightShooter | LB=Feeder");
        telemetry.addData("leftShooter dir",  leftShooter  == null ? "not found" : (leftRev  ? "REVERSE" : "FORWARD"));
        telemetry.addData("rightShooter dir", rightShooter == null ? "not found" : (rightRev ? "REVERSE" : "FORWARD"));
        telemetry.addData("Feeder dir",       feeder       == null ? "not found" : (feederRev ? "REVERSE" : "FORWARD"));
    }
}