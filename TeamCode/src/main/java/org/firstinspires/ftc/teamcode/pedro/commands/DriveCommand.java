package org.firstinspires.ftc.teamcode.pedro.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedro.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.subsystems.PinpointOdometry;
import org.firstinspires.ftc.teamcode.pedro.subsystems.AprilTagVision;

public class DriveCommand implements Command {
    private final MecanumDrive drive;
    private final PinpointOdometry odometry;
    private final Gamepad gamepad;
    private final AprilTagVision vision;
    private boolean slow, lastBumper, lastBack;
    public DriveCommand(MecanumDrive drive, PinpointOdometry odometry, Gamepad gamepad, AprilTagVision vision) {
        this.drive = drive; this.odometry = odometry; this.gamepad = gamepad; this.vision = vision;
    }
    @Override public void execute() {
        odometry.update();
        if (gamepad.back && !lastBack) odometry.reset();
        lastBack = gamepad.back;
        if (gamepad.right_bumper && !lastBumper) slow = !slow;
        lastBumper = gamepad.right_bumper;
        double f = -gamepad.left_stick_y, l = -gamepad.left_stick_x;
        double h = odometry.getHeading();
        boolean aiming = gamepad.right_trigger > 0.1;
        double correction = vision.update(aiming);
        double turn = aiming ? correction : -gamepad.right_stick_x;
        drive.drive(f*Math.cos(h) + l*Math.sin(h), -f*Math.sin(h) + l*Math.cos(h),
                turn, slow ? 0.5 : 1.0);
    }
    @Override public void end(boolean interrupted) { drive.stop(); }
}
