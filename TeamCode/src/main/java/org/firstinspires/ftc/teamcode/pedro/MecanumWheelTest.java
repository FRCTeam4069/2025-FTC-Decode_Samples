package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.subsystems.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Isolate one drive motor at a time, with TeleOp directions and no localization. */
@TeleOp(name = "Mecanum Wheel Test", group = "Tuning")
public class MecanumWheelTest extends LinearOpMode {
    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        telemetry.addLine("Lift the robot safely so all wheels are clear.");
        telemetry.addLine("Hold one button: X = front left, Y = front right,");
        telemetry.addLine("A = rear left, B = rear right. Each should roll forward.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                int count = (gamepad1.x ? 1 : 0) + (gamepad1.y ? 1 : 0)
                        + (gamepad1.a ? 1 : 0) + (gamepad1.b ? 1 : 0);
                int wheel = count != 1 ? -1 : gamepad1.x ? 0 : gamepad1.y ? 1 : gamepad1.a ? 2 : 3;
                drive.testWheel(wheel, 0.2);
                telemetry.addData("Wheel", wheel == 0 ? "front left" : wheel == 1 ? "front right"
                        : wheel == 2 ? "rear left" : wheel == 3 ? "rear right" : "stopped");
                telemetry.addLine("X=front left  Y=front right  A=rear left  B=rear right");
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
