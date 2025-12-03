package org.firstinspires.ftc.teamcode.pedroPathing.auto.Far;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Far Leave", group = "Autonomous")
@Configurable
public class BlueFarLeave extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90))); // Your starting pose

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // MUST be called every loop
        pathState = autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain FarShot;
        public PathChain Leave;

        public Paths(Follower follower) {
            FarShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(58.083, 14.590),
                                    new Pose(62.498, 13.196)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-67))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.498, 13.196),
                                    new Pose(69.700, 23.884),
                                    new Pose(59.710, 35.965)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-67), Math.toRadians(180))
                    .build();
        }
    }
    /***
     * Working state machine that actually runs your paths.
     */
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start FarShot path
                follower.followPath(paths.FarShot);
                pathState = 1;
                break;

            case 1:
                // When FarShot finishes, start Leave path
                if (!follower.isBusy()) {
                    follower.followPath(paths.Leave);
                    pathState = 2;
                }
                break;

            case 2:
                // Done with Leave path
                if (!follower.isBusy()) {
                    pathState = 3;
                }
                break;

            case 3:
                // Auto finished
                break;
        }

        return pathState;
    }
}
