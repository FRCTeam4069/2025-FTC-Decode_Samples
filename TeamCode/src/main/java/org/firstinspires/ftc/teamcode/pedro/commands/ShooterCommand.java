package org.firstinspires.ftc.teamcode.pedro.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

public class ShooterCommand implements Command {
    private final Shooter shooter;
    private final Gamepad gamepad;
    private final ElapsedTime timer = new ElapsedTime();
    private double targetRpm = 3000;
    private boolean up, down, left, right;
    public ShooterCommand(Shooter shooter, Gamepad gamepad) { this.shooter=shooter; this.gamepad=gamepad; }
    @Override public void initialize() { timer.reset(); }
    @Override public void execute() {
        if (gamepad.dpad_up && !up) targetRpm += 100;
        if (gamepad.dpad_down && !down) targetRpm -= 100;
        if (gamepad.dpad_right && !right) targetRpm += 10;
        if (gamepad.dpad_left && !left) targetRpm -= 10;
        up=gamepad.dpad_up; down=gamepad.dpad_down; right=gamepad.dpad_right; left=gamepad.dpad_left;
        targetRpm = Range.clip(targetRpm,0,Shooter.MAX_RPM);
        double dt = timer.seconds(); timer.reset();
        shooter.update(gamepad.right_trigger > 0.1, targetRpm, dt);
    }
    public double getTargetRpm() { return targetRpm; }
    @Override public void end(boolean interrupted) { shooter.stop(); }
}
