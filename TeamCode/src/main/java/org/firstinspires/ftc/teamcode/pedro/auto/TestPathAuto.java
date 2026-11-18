package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.commands.Scheduler;
import org.firstinspires.ftc.teamcode.pedro.commands.TestPathCommand;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;

@Autonomous(name = "Pedro Test Path", group = "Autonomous")
public class TestPathAuto extends LinearOpMode {
    @Override public void runOpMode() {
        Intake intake = new Intake(hardwareMap);
        Follower follower = Constants.create(hardwareMap);
        TestPaths paths = new TestPaths();
        Scheduler scheduler = new Scheduler();
        TestPathCommand command = new TestPathCommand(follower, intake, paths);
        telemetry.addLine("Keep robot still while Pinpoint calibrates.");
        telemetry.addLine("Foresight gains and odometry must be tuned.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        try {
            follower.setPose(paths.start);
            scheduler.schedule(command);
            while (opModeIsActive()) {
                scheduler.run();
                telemetry.addData("State", command.getState());
                telemetry.addData("Pose", follower.pose());
                telemetry.update();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            intake.stop();
            follower.stop();
            follower.drivetrain.stop();
        }
    }
}