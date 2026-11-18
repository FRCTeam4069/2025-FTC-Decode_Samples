package org.firstinspires.ftc.teamcode.pedro.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.auto.TestPaths;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;

/** Run path one, then path two with the intake on; stop all outputs on exit. */
public class TestPathCommand implements Command {
    private static final double PATH_TIMEOUT_SECONDS = 15;
    private final Follower follower;
    private final Intake intake;
    private final TestPaths paths;
    private final ElapsedTime timer = new ElapsedTime();
    private int state; // 1 first path, 2 second path, 3 complete, 4 timed out
    public TestPathCommand(Follower follower, Intake intake, TestPaths paths) {
        this.follower = follower; this.intake = intake; this.paths = paths;
    }
    @Override public void initialize() {
        state = 1;
        intake.stop();
        follower.follow(paths.first());
        timer.reset();
    }
    @Override public void execute() {
        follower.update();
        if (timer.seconds() > PATH_TIMEOUT_SECONDS) {
            state = 4;
        } else if (!follower.following()) {
            if (state == 1) {
                intake.setPower(1.0);
                follower.follow(paths.second());
                state = 2;
                timer.reset();
            } else if (state == 2) {
                state = 3;
            }
        }
    }
    @Override public boolean isFinished() { return state >= 3; }
    @Override public void end(boolean interrupted) {
        intake.stop();
        follower.stop();
        // Pedro 3.0.1 stop() switches modes; explicitly zero the motors now.
        follower.drivetrain.stop();
    }
    public String getState() {
        switch (state) {
            case 1: return "Path 1";
            case 2: return "Path 2: intake on";
            case 3: return "Complete";
            case 4: return "Timed out; motors stopped";
            default: return "Ready";
        }
    }
}
