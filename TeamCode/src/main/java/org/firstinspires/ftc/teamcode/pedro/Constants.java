package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.subsystems.PinpointOdometry;
import org.firstinspires.ftc.teamcode.pedro.subsystems.RobotMecanum;

/** Pedro 3 configuration; Foresight controller gains are untuned starting values. */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(MecanumDrive.FRONT_LEFT);
        c.frontRightName.set(MecanumDrive.FRONT_RIGHT);
        c.backLeftName.set(MecanumDrive.BACK_LEFT);
        c.backRightName.set(MecanumDrive.BACK_RIGHT);
        c.frontLeftDirection.set(MecanumDrive.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(MecanumDrive.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(MecanumDrive.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(MecanumDrive.BACK_RIGHT_DIRECTION);
    });

    // These gains and braking coefficients are examples, not measured on this robot.
    // Replace with ForesightTuner output before relying on autonomous accuracy.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.proportional(0.1));
        c.strafeTranslational.set(Controller.proportional(0.1));
        c.headingFeedback.set(Controller.proportional(2.0));
        c.coast.set(Controller.proportionalFeedforward(0.011));
        c.brake.set(Controller.proportionalFeedforward(0.0087));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056, 0.0064));
        c.linearBrakeCoefficients.set(Matrix.diag(0.106, 0.087));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00147, 0.00138));
        // Prior robot's estimated velocities and natural decelerations.
        c.maxAchievableForwardVelocity.set(62.0);
        c.maxAchievableStrafeVelocity.set(37.0);
        c.naturalForwardDeceleration.set(64.0);
        c.naturalStrafeDeceleration.set(62.0);
        c.maxVelocityConstraint.set(20.0);
    });
    public static Follower create(HardwareMap map) {
        return new Follower(new PinpointLocalizer(map, PinpointOdometry.createConfig()),
                new RobotMecanum(map, drivetrainConfig), new Foresight(foresightConfig));
    }
}
