package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.subsystems.PinpointOdometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Push by hand; this diagnostic never powers the drive motors. */
@TeleOp(name = "Pinpoint Localization Test", group = "Tuning")
public class PinpointLocalizationTest extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        PinpointOdometry localizer = new PinpointOdometry(hardwareMap);
        telemetry.addLine("Keep stationary; press START to set origin.");
        telemetry.addLine("Push forward, left, then turn CCW by hand.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        localizer.reset();
        while (opModeIsActive()) {
            localizer.update();
            telemetry.addData("X encoder", localizer.getXTicks());
            telemetry.addData("Y encoder", localizer.getYTicks());
            telemetry.addData("Pinpoint degrees", "%.1f", Math.toDegrees(localizer.getHeading()));
            telemetry.update();
            sleep(50);
        }
    }
}
