package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Full Power Test", group = "Test")
public class FullPowerTest extends LinearOpMode {

    // ===== EDIT THESE =====
    private static final boolean LEFT_SHOOTER_REVERSED  = true;
    private static final boolean RIGHT_SHOOTER_REVERSED = false;
    private static final boolean FEEDER_REVERSED        = false;
    private static final double  POWERLeft = -1.0;
    private static final double  POWERRight = -1.0;
    private static final double  POWERFeeder = -1.0;
    private static final double  POWERIntake = -1.0;

    // ======================

    @Override
    public void runOpMode() {
        // tryGet returns null if a motor isn't in the config, so you can add them one at a time
        DcMotor leftShooter  = hardwareMap.tryGet(DcMotor.class, "leftShooter");
        DcMotor rightShooter = hardwareMap.tryGet(DcMotor.class, "rightShooter");
        DcMotor feeder       = hardwareMap.tryGet(DcMotor.class, "Feeder");
        DcMotor intake       = hardwareMap.tryGet(DcMotor.class, "Intake");

        if (leftShooter != null) {
            leftShooter.setDirection(LEFT_SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightShooter != null) {
            rightShooter.setDirection(RIGHT_SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (feeder != null) {
            feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (intake != null) {
            intake.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Hold A on gamepad1 = all motors at full power");
        telemetry.addData("leftShooter", leftShooter == null ? "not found" : "found");
        telemetry.addData("rightShooter", rightShooter == null ? "not found" : "found");
        telemetry.addData("Feeder", feeder == null ? "not found" : "found");
        telemetry.addData("Intake", intake == null ? "not found" : "found");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double l = gamepad1.a ? POWERLeft : 0;
            double r = gamepad1.a ? POWERRight : 0;
            double f = gamepad1.a ? POWERFeeder : 0;
            double i = gamepad1.a ? POWERIntake : 0;

            if (leftShooter != null)  leftShooter.setPower(l);
            if (rightShooter != null) rightShooter.setPower(r);
            if (feeder != null)       feeder.setPower(f);

            telemetry.addData("A held", gamepad1.a);
            telemetry.addData("Power Left", l);
            telemetry.addData("Power Right", r);
            telemetry.addData("Power Feeder", f);
            telemetry.addData("Power Intake", i);
            telemetry.update();
        }

        if (leftShooter != null)  leftShooter.setPower(0);
        if (rightShooter != null) rightShooter.setPower(0);
        if (feeder != null)       feeder.setPower(0);
        if (intake != null)       intake.setPower(0);
    }
}