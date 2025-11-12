package org.firstinspires.ftc.teamcode.pedroPathing.auto;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Blue Close Launch", group = "Autonomous")
@Configurable
public class BlueCloseLaunch extends OpMode {


    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;


    private DcMotor intakeMotor;
    private DcMotor shooterMotor;


    private boolean waiting = false;
    private double waitStartTime = 0;
    private boolean pathStarted = false;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        follower = Constants.createFollower(hardwareMap);


        // Starting position from the online vizualiser tool
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(-36)));


        paths = new Paths(follower);


        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setPower(0);
        // shooterMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse line if needed


        pathState = 0;
        waiting = false;
        pathStarted = false;


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();


        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Waiting", waiting);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading°", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {


        public PathChain MovetoFirstShot;
        public PathChain MovetoPickup;
        public PathChain Pickup;
        public PathChain Movetosecondshot;
        public PathChain MovetoPickUpTwo;
        public PathChain PickUpTwo;
        public PathChain MovetoShootThree;
        public PathChain Movetopark;


        public Paths(Follower follower) {
            MovetoFirstShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 123.000), new Pose(48.558, 104.968))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-36))
                    .build();

            MovetoPickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.558, 104.968),
                                    new Pose(51.113, 91.957),
                                    new Pose(45.537, 83.593)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))
                    .build();

            Pickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.537, 83.593), new Pose(29.739, 83.593))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Movetosecondshot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(29.739, 83.593),
                                    new Pose(34.850, 93.351),
                                    new Pose(47.861, 96.604)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            MovetoPickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.861, 96.604),
                                    new Pose(58.316, 88.240),
                                    new Pose(70.862, 59.895),
                                    new Pose(45.537, 59.663)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            PickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.537, 59.663), new Pose(32.748, 59.690))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShootThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(32.748, 59.690),
                                    new Pose(45.770, 66.865),
                                    new Pose(47.396, 96.604)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            Movetopark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.396, 96.604),
                                    new Pose(69.677, 75.716),
                                    new Pose(61.781, 36.232),
                                    new Pose(80.361, 35.768),
                                    new Pose(29.032, 35.768)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }


    private void startPause() {
        waitStartTime = getRuntime();
        waiting = true;
        pathStarted = false;
        intakeMotor.setPower(1.0); //Turn on the intake to feed the shooter
        shooterMotor.setPower(1.0); // Start spinning the shooter while paused
        // shooterMotor.setPower(-1.0); // reverse the Shooter (leave commented)
    }


    private void stopRobot() {
        follower.setTeleOpDrive(0, 0, 0, true);
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
        //This is only used at the end and all motors should be off
    }


    public void autonomousPathUpdate() {
        if (waiting) {
            stopRobot();
            if (getRuntime() - waitStartTime >= 3.0) { // 3 seconds
                waiting = false;
                intakeMotor.setPower(0); // Intake after pause
                shooterMotor.setPower(0); // Stop shooter after pause
                pathState++;
            } else return;
        }


        if (pathState > 8) {
            stopRobot();
            return;
        }


        switch (pathState) {


            case 0:
                if (!pathStarted) { follower.followPath(paths.MovetoFirstShot); pathStarted = true; shooterMotor.setPower(1.0); }
                else if (!follower.isBusy()) startPause();
                break;


            case 1:
                if (!pathStarted) { follower.followPath(paths.MovetoPickup); pathStarted = true; }
                else if (!follower.isBusy()) { pathState++; pathStarted = false; }
                break;


            case 2:
                if
                (!pathStarted) { follower.followPath(paths.Pickup); pathStarted = true; intakeMotor.setPower(-1); }
                else if
                (!follower.isBusy()) {intakeMotor.setPower(0); pathState++; pathStarted = false; }
                break;


            case 3:
                if (!pathStarted) { follower.followPath(paths.Movetosecondshot); pathStarted = true; shooterMotor.setPower(1); }
                else if (!follower.isBusy()) startPause();
                break;


            case 4:
                if (!pathStarted) { follower.followPath(paths.MovetoPickUpTwo); pathStarted = true;  }
                else if (!follower.isBusy()) { pathState++; pathStarted = false; }
                break;


            case 5:
                if
                (!pathStarted) { follower.followPath(paths.PickUpTwo); pathStarted = true; intakeMotor.setPower(-1.0); }
                else if
                (!follower.isBusy()) { intakeMotor.setPower(0); pathState++; pathStarted = false; }
                break;


            case 6:
                if (!pathStarted) { follower.followPath(paths.MovetoShootThree); pathStarted = true; shooterMotor.setPower(1); }
                else if (!follower.isBusy()) startPause();
                break;


            case 7:
                if (!pathStarted) { follower.followPath(paths.Movetopark); pathStarted = true; shooterMotor.setPower(0); }
                else if (!follower.isBusy()) { stopRobot(); pathState++; }
                break;
        }
    }
}
