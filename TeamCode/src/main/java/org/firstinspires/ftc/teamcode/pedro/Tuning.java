package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.subsystems.PinpointOdometry;
import org.firstinspires.ftc.teamcode.pedro.subsystems.RobotMecanum;

public class Tuning {
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(map -> new PinpointLocalizer(map, PinpointOdometry.createConfig()),
                map -> new RobotMecanum(map, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
