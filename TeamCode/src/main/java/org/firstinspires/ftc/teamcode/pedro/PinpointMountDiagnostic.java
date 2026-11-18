package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.subsystems.PinpointOdometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Display the same corrected encoders and heading used by the TeleOp. */
@TeleOp(name = "Pinpoint Mount Diagnostic", group = "Tuning")
public class PinpointMountDiagnostic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointOdometry localizer = new PinpointOdometry(hardwareMap);
        telemetry.addLine("Keep the robot still and press START to zero readings.");
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
