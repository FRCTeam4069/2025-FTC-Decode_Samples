package org.firstinspires.ftc.teamcode.pedro.commands;

/** A reusable action called by the scheduler once per robot loop. */
public interface Command {
    default void initialize() { }
    void execute();
    default boolean isFinished() { return false; }
    default void end(boolean interrupted) { }
}
