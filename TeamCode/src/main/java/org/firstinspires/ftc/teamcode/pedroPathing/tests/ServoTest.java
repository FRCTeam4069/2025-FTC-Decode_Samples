package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="motor67", group="Concept")
public class ServoTest extends LinearOpMode {

    private CRServo motor67 = null;

    @Override
    public void runOpMode() {

        motor67 = hardwareMap.get(CRServo.class, "motor67");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double servoPower = 0.0;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servoPower = 1.0;
            } else if (gamepad1.dpad_down) {
                servoPower = -1.0;
            } else if (gamepad1.dpad_right) {
                servoPower = 0.0;
            }
            motor67.setPower(servoPower);

            telemetry.addData("Servo Power", servoPower);
            telemetry.update();
        }
    }
}
