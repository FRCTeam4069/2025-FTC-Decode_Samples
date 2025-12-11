package org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class RedFourCyclesPaths {

    public static class Paths {

        // Poses
        public Pose moveToFirstShotStart = new Pose(126.116, 123.000);
        public Pose moveToFirstShotEnd   = new Pose(95.226, 99.174);

        public Pose moveToPickupStart    = new Pose(95.226, 99.160);
        public Pose moveToPickupControl1 = new Pose(91.277, 92.439);
        public Pose moveToPickupEnd      = new Pose(96, 84.310);

        public Pose pickupStart          = new Pose(96, 84.310);
        public Pose pickupEnd            = new Pose(120.774, 84.077);

        public Pose moveToSecondShotStart = new Pose(120.774, 84.077);
        public Pose moveToSecondShotControl1 = new Pose(115.200, 97.548);
        public Pose moveToSecondShotEnd   = new Pose(95.226, 99.174);

        public Pose moveToPickUpTwoStart   = new Pose(95.226, 99.174);
        public Pose moveToPickUpTwoControl1 = new Pose(85.006, 71.303);
        public Pose moveToPickUpTwoEnd     = new Pose(96, 57.5);

        public Pose pickUpTwoStart = new Pose(96, 57.5);
        public Pose pickUpTwoEnd   = new Pose(122.865, 57.5);

        public Pose moveToShootThreeStart = new Pose(122.865, 58.529);
        public Pose moveToShootThreeControl1 = new Pose(108.929, 61.548);
        public Pose moveToShootThreeEnd = new Pose(95.226, 99.174);

        public Pose moveToPickUpThreeStart = new Pose(95.226, 99.406);
        public Pose moveToPickUpThreeEnd = new Pose(98.942, 59.458);

        // Headings (in degrees, configurable)
        public double moveToFirstShotStartHeading = -143;
        public double moveToFirstShotEndHeading   = -143;

        public double moveToPickupStartHeading = -143;
        public double moveToPickupEndHeading   = 0;

        public double pickupStartHeading = 0;
        public double pickupEndHeading   = 0;

        public double moveToSecondShotStartHeading = 0;
        public double moveToSecondShotEndHeading   = -143;

        public double moveToPickUpTwoStartHeading = -143;
        public double moveToPickUpTwoEndHeading   = 0;

        public double pickUpTwoStartHeading = 0;
        public double pickUpTwoEndHeading   = 0;

        public double moveToShootThreeStartHeading = 0;
        public double moveToShootThreeEndHeading   = -143;

        public double moveToPickUpThreeStartHeading = -143;
        public double moveToPickUpThreeEndHeading   = 0;

        // Paths
        public PathChain MovetoFirstShot;
        public PathChain MovetoPickup;
        public PathChain Pickup;
        public PathChain Movetosecondshot;
        public PathChain MovetoPickUpTwo;
        public PathChain PickUpTwo;
        public PathChain MovetoShootThree;
        public PathChain MovetoPickUpThree;

        public Paths(Follower follower) {
            buildPaths(follower);
        }

        public void buildPaths(Follower follower) {

            MovetoFirstShot = follower.pathBuilder()
                    .addPath(new BezierLine(moveToFirstShotStart, moveToFirstShotEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToFirstShotStartHeading),
                            Math.toRadians(moveToFirstShotEndHeading))
                    .build();

            MovetoPickup = follower.pathBuilder()
                    .addPath(new BezierCurve(moveToPickupStart, moveToPickupControl1, moveToPickupEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToPickupStartHeading),
                            Math.toRadians(moveToPickupEndHeading))
                    .build();

            Pickup = follower.pathBuilder()
                    .addPath(new BezierLine(pickupStart, pickupEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(pickupStartHeading),
                            Math.toRadians(pickupEndHeading))
                    .build();

            Movetosecondshot = follower.pathBuilder()
                    .addPath(new BezierCurve(moveToSecondShotStart, moveToSecondShotControl1, moveToSecondShotEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToSecondShotStartHeading),
                            Math.toRadians(moveToSecondShotEndHeading))
                    .build();

            MovetoPickUpTwo = follower.pathBuilder()
                    .addPath(new BezierCurve(moveToPickUpTwoStart, moveToPickUpTwoControl1, moveToPickUpTwoEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToPickUpTwoStartHeading),
                            Math.toRadians(moveToPickUpTwoEndHeading))
                    .build();

            PickUpTwo = follower.pathBuilder()
                    .addPath(new BezierLine(pickUpTwoStart, pickUpTwoEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(pickUpTwoStartHeading),
                            Math.toRadians(pickUpTwoEndHeading))
                    .build();

            MovetoShootThree = follower.pathBuilder()
                    .addPath(new BezierCurve(moveToShootThreeStart, moveToShootThreeControl1, moveToShootThreeEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToShootThreeStartHeading),
                            Math.toRadians(moveToShootThreeEndHeading))
                    .build();

            MovetoPickUpThree = follower.pathBuilder()
                    .addPath(new BezierLine(moveToPickUpThreeStart, moveToPickUpThreeEnd))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}