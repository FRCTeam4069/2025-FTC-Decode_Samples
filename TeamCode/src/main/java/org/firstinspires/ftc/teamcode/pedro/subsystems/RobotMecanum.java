package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Pedro drivetrain with the same strafe/turn mapping as the TeleOp. */
public class RobotMecanum extends Mecanum {
    public RobotMecanum(HardwareMap map, MecanumConfig config) {
        super(map, config);
    }

    @Override
    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
        return super.computeWheelPowersUnnormalized(
                new DrivePowers(powers.forward(), powers.turn(), powers.strafe()));
    }
}
