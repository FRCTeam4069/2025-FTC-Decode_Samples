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

        // Poses (Mirrored)
        public Pose moveToFirstShotStart = new Pose(21.0, 21.0);
        public Pose moveToFirstShotEnd   = new Pose(48.093, 44.84);

        public Pose moveToPickupStart    = new Pose(48.093, 44.84);
        public Pose moveToPickupControl1 = new Pose(58.316, 51.81);
        public Pose moveToPickupControl2 = new Pose(58.083, 62.498);
        public Pose moveToPickupEnd      = new Pose(53.901, 60.407);

        public Pose pickupStart = new Pose(53.901, 60.407);
        public Pose pickupEnd   = new Pose(26.00, 60.407);

        public Pose moveToSecondShotStart = new Pose(29.739, 60.407);
        public Pose moveToSecondShotControl1 = new Pose(34.85, 50.649);
        public Pose moveToSecondShotEnd   = new Pose(48.325, 49.719);

        public Pose moveToPickUpTwoStart   = new Pose(48.325, 49.719);
        public Pose moveToPickUpTwoControl1 = new Pose(58.316, 55.76);
        public Pose moveToPickUpTwoControl2 = new Pose(71.094, 84.569);
        public Pose moveToPickUpTwoEnd     = new Pose(55.295, 88.0);

        public Pose pickUpTwoStart = new Pose(55.295, 88.0);
        public Pose pickUpTwoEnd   = new Pose(22.00, 88.0);

        public Pose moveToShootThreeStart = new Pose(32.759, 85.034);
        public Pose moveToShootThreeControl1 = new Pose(45.77, 77.135);
        public Pose moveToShootThreeEnd = new Pose(49.487, 46.234);

        public Pose moveToPickUpThreeStart = new Pose(49.487, 46.234);
        public Pose moveToPickUpThreeControl1 = new Pose(69.003, 69.932);
        public Pose moveToPickUpThreeControl2 = new Pose(65.518, 108.964);
        public Pose moveToPickUpThreeEnd = new Pose(57.386, 109.429);

        public Pose pickUpThreeStart = new Pose(57.386, 109.429);
        public Pose pickUpThreeEnd   = new Pose(29.042, 109.661);

        public Pose moveToShootFourStart = new Pose(29.042, 109.661);
        public Pose moveToShootFourControl1 = new Pose(10.223, 105.944);
        public Pose moveToShootFourControl2 = new Pose(56.689, 73.417);
        public Pose moveToShootFourEnd = new Pose(49.022, 46.699);

        // Headings (Mirrored: 180 - original)
        public double moveToFirstShotStartHeading = 54;   // 180 - 216
        public double moveToFirstShotEndHeading   = 54;   // 180 - 222

        public double moveToPickupStartHeading = -36;     // 180 - 216
        public double moveToPickupEndHeading   = 180;     // 180 - 0

        public double pickupStartHeading = 180;           // 180 - 0
        public double pickupEndHeading   = 180;           // 180 - 0

        public double moveToSecondShotStartHeading = 54; // 180 - 0
        public double moveToSecondShotEndHeading   = 54; // 180 - 225

        public double moveToPickUpTwoStartHeading = 180;  // 180 - 0
        public double moveToPickUpTwoEndHeading   = 180;  // 180 - 0

        public double pickUpTwoStartHeading = 180;        // 180 - 0
        public double pickUpTwoEndHeading   = 180;        // 180 - 0

        public double moveToShootThreeStartHeading = 180; // 180 - 0
        public double moveToShootThreeEndHeading   = -45; // 180 - 225

        public double moveToPickUpThreeStartHeading = -45; // 180 - 225
        public double moveToPickUpThreeEndHeading   = 180; // 180 - 0

        public double pickUpThreeStartHeading = 180;      // 180 - 0
        public double pickUpThreeEndHeading   = 180;      // 180 - 0

        public double moveToShootFourStartHeading = 180;  // 180 - 0
        public double moveToShootFourEndHeading   = -45;  // 180 - 225


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
