package org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class BlueFourCyclesPaths {

    public static class Paths {

        // Poses
        public Pose moveToFirstShotStart = new Pose(21.0, 123.0);
        public Pose moveToFirstShotEnd   = new Pose(48.093, 99.16);

        public Pose moveToPickupStart    = new Pose(48.093, 99.16);
        public Pose moveToPickupControl1 = new Pose(58.316, 92.19);
        public Pose moveToPickupControl2 = new Pose(58.083, 81.502);
        public Pose moveToPickupEnd      = new Pose(53.901, 83.593);

        public Pose pickupStart           = new Pose(53.901, 83.593);
        public Pose pickupEnd             = new Pose(26.00, 83.593);

        public Pose moveToSecondShotStart = new Pose(29.739, 83.593);
        public Pose moveToSecondShotControl1 = new Pose(34.85, 93.351);
        public Pose moveToSecondShotEnd   = new Pose(48.325, 94.281);

        public Pose moveToPickUpTwoStart   = new Pose(48.325, 94.281);
        public Pose moveToPickUpTwoControl1 = new Pose(58.316, 88.24);
        public Pose moveToPickUpTwoControl2 = new Pose(71.094, 59.431);
        public Pose moveToPickUpTwoEnd     = new Pose(55.295, 56.5);

        public Pose pickUpTwoStart = new Pose(55.295, 56.5);
        public Pose pickUpTwoEnd   = new Pose(22.00, 56.5);

        public Pose moveToShootThreeStart = new Pose(32.759, 58.966);
        public Pose moveToShootThreeControl1 = new Pose(45.77, 66.865);
        public Pose moveToShootThreeEnd = new Pose(49.487, 97.766);

        public Pose moveToPickUpThreeStart = new Pose(49.487, 97.766);
        public Pose moveToPickUpThreeControl1 = new Pose(69.003, 74.068);
        public Pose moveToPickUpThreeControl2 = new Pose(65.518, 35.036);
        public Pose moveToPickUpThreeEnd = new Pose(57.386, 34.571);

        public Pose pickUpThreeStart = new Pose(57.386, 34.571);
        public Pose pickUpThreeEnd   = new Pose(29.042, 34.339);

        public Pose moveToShootFourStart = new Pose(29.042, 34.339);
        public Pose moveToShootFourControl1 = new Pose(10.223, 38.056);
        public Pose moveToShootFourControl2 = new Pose(56.689, 70.583);
        public Pose moveToShootFourEnd = new Pose(49.022, 97.301);

        // Headings (in degrees, configurable)
        public double moveToFirstShotStartHeading = -36;
        public double moveToFirstShotEndHeading   = -42;

        public double moveToPickupStartHeading = -36;
        public double moveToPickupEndHeading   = 180;

        public double pickupStartHeading = 180;
        public double pickupEndHeading   = 180;

        public double moveToSecondShotStartHeading = 180;
        public double moveToSecondShotEndHeading   = -42;

        public double moveToPickUpTwoStartHeading = 180;
        public double moveToPickUpTwoEndHeading   = 180;

        public double pickUpTwoStartHeading = 180;
        public double pickUpTwoEndHeading   = 180;

        public double moveToShootThreeStartHeading = 180;
        public double moveToShootThreeEndHeading   = -45;

        public double moveToPickUpThreeStartHeading = -45;
        public double moveToPickUpThreeEndHeading   = 180;

        public double pickUpThreeStartHeading = 180;
        public double pickUpThreeEndHeading   = 180;

        public double moveToShootFourStartHeading = 180;
        public double moveToShootFourEndHeading   = -45;

        // Paths
        public PathChain MovetoFirstShot;
        public PathChain MovetoPickup;
        public PathChain Pickup;
        public PathChain Movetosecondshot;
        public PathChain MovetoPickUpTwo;
        public PathChain PickUpTwo;
        public PathChain MovetoShootThree;
        public PathChain MovetoPickUpThree;
        public PathChain PickUpThree;
        public PathChain MovetoShootFour;

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
                    .addPath(new BezierCurve(moveToPickupStart, moveToPickupControl1, moveToPickupControl2, moveToPickupEnd))
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
                    .addPath(new BezierCurve(moveToPickUpTwoStart, moveToPickUpTwoControl1, moveToPickUpTwoControl2, moveToPickUpTwoEnd))
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
                    .addPath(new BezierCurve(moveToPickUpThreeStart, moveToPickUpThreeControl1, moveToPickUpThreeControl2, moveToPickUpThreeEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToPickUpThreeStartHeading),
                            Math.toRadians(moveToPickUpThreeEndHeading))
                    .build();

            PickUpThree = follower.pathBuilder()
                    .addPath(new BezierLine(pickUpThreeStart, pickUpThreeEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(pickUpThreeStartHeading),
                            Math.toRadians(pickUpThreeEndHeading))
                    .build();

            MovetoShootFour = follower.pathBuilder()
                    .addPath(new BezierCurve(moveToShootFourStart, moveToShootFourControl1, moveToShootFourControl2, moveToShootFourEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(moveToShootFourStartHeading),
                            Math.toRadians(moveToShootFourEndHeading))
                    .build();
        }
    }
}
