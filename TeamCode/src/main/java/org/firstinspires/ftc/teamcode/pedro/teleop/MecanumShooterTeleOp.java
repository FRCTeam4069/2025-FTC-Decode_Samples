package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.commands.*;
import org.firstinspires.ftc.teamcode.pedro.subsystems.*;

@TeleOp(name = "Mecanum Shooter TeleOp", group = "TeleOp")
public class MecanumShooterTeleOp extends LinearOpMode {
    @Override public void runOpMode() {
        Intake intake = new Intake(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
        AprilTagVision vision = new AprilTagVision(hardwareMap);
        Scheduler scheduler = new Scheduler();
        ShooterCommand shooterCommand = new ShooterCommand(shooter, gamepad1);
        telemetry.addLine("Keep robot still. Ready; press START."); telemetry.update();
        try {
            waitForStart();
            if (isStopRequested()) return;
            odometry.reset();
            scheduler.schedule(new DriveCommand(drive, odometry, gamepad1, vision));
            scheduler.schedule(shooterCommand);
            scheduler.schedule(new IntakeCommand(intake, () -> gamepad1.a, () -> gamepad1.b));
            while (opModeIsActive()) {
                scheduler.run();
                shooter.addTelemetry(telemetry, shooterCommand.getTargetRpm());
                telemetry.addData("Intake power", "%.1f", intake.getPower());
                telemetry.addData("Pinpoint status", odometry.getStatus());
                telemetry.addData("Pose X / Y (in)", "%.1f / %.1f", odometry.getX(), odometry.getY());
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(odometry.getHeading()));
                if (gamepad1.right_trigger > 0.1) {
                    telemetry.addData("Tag aiming", vision.isVisible() ? "Tracking" : "No fresh target");
                    if (vision.isVisible()) {
                        telemetry.addData("Tag ID", vision.getLockedId());
                        telemetry.addData("Tag error (deg)", "%.1f", vision.getErrorDegrees());
                    }
                }
                telemetry.update();
            }
        } finally { scheduler.cancelAll(); drive.stop(); shooter.stop(); intake.stop(); vision.stop(); }
    }
}
