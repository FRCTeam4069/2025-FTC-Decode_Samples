package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** The two paths exported from the Pedro visualizer. */
public class TestPaths {
    private final PoseFactory degrees = PoseFactory.degrees();
    public final Pose start = degrees.of(56, 8, 90);
    private final Pose firstEnd = degrees.of(27.1104, 45.8264, 180);
    private final Pose secondEnd = degrees.of(21.9675, 103.8466, -84.9346);
    public Path first() { return Paths.line(start, firstEnd).linear(start, firstEnd); }
    public Path second() { return Paths.line(firstEnd, secondEnd).reverseTangent(); }
}
