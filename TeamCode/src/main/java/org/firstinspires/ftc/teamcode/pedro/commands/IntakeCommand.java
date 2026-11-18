package org.firstinspires.ftc.teamcode.pedro.commands;

import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommand implements Command {
    private final Intake intake;
    private final BooleanSupplier forward, reverse;
    public IntakeCommand(Intake intake, BooleanSupplier forward, BooleanSupplier reverse) {
        this.intake = intake; this.forward = forward; this.reverse = reverse;
    }
    @Override public void execute() {
        intake.setPower((forward.getAsBoolean() ? 1 : 0) - (reverse.getAsBoolean() ? 1 : 0));
    }
    @Override public void end(boolean interrupted) { intake.stop(); }
}
