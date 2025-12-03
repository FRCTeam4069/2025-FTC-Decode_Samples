package org.firstinspires.ftc.teamcode.pedroPathing.auto.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFourCyclesPaths {

    public static class Paths {

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
            MovetoFirstShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.369, 122.393), new Pose(95.721, 95.442))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-143))
                    .build();

            MovetoPickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.721, 95.442),
                                    new Pose(94.792, 94.281),
                                    new Pose(89.216, 83.361),
                                    new Pose(101.297, 82.664)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(0))
                    .build();

            Pickup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.297, 82.664), new Pose(115.470, 81.502))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Movetosecondshot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(115.470, 81.502),
                                    new Pose(110.126, 94.048),
                                    new Pose(94.792, 94.745)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();

            MovetoPickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.792, 94.745),
                                    new Pose(77.135, 74.300),
                                    new Pose(74.347, 59.198),
                                    new Pose(92.933, 58.966)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            PickUpTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.933, 58.966), new Pose(110.823, 58.501))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShootThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(110.823, 58.501),
                                    new Pose(70.165, 64.774),
                                    new Pose(95.024, 95.210)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();

            MovetoPickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.024, 95.210),
                                    new Pose(82.943, 55.713),
                                    new Pose(74.347, 36.197),
                                    new Pose(89.913, 34.571)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(0))
                    .build();

            PickUpThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(89.913, 34.571), new Pose(116.399, 33.874))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            MovetoShootFour = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(116.399, 33.874),
                                    new Pose(134.753, 38.753),
                                    new Pose(76.438, 72.674),
                                    new Pose(94.792, 95.210)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-143))
                    .build();
        }
    }

}
