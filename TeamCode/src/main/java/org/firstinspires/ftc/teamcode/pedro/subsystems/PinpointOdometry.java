package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.OptionalDouble;

/** Pedro Pinpoint localizer for two pods and a sticker-up Pinpoint. */
public class PinpointOdometry {
    // Preliminary values: measure each pod relative to the center of rotation.
    public static final double X_POD_Y_INCHES = 0.0; // forward pod on the centerline
    public static final double Y_POD_X_INCHES = -1.0; // strafe pod is behind
    // Retained from the previous code; tune for the actual pod model.
    public static final double INCHES_PER_TICK = 0.001989436789;
    // Both raw encoders decreased in the positive direction during the prior test.
    public static final int X_DIRECTION = -1;
    public static final int Y_DIRECTION = -1;

    private final PinpointLocalizer localizer;
    private final GoBildaPinpointDriver pinpoint;
    private int lastX, lastY;
    private long xTicks, yTicks;
    private double lastHeading, heading;
    private boolean initialized;

    public static PinpointConfig createConfig() {
        return new PinpointConfig(c -> {
            c.name.set("pinpoint");
            c.xPodOffset.set(X_POD_Y_INCHES);
            c.yPodOffset.set(Y_POD_X_INCHES);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.encoderResolutionUnit.set(DistanceUnit.INCH);
            c.ticksPerUnit.set(OptionalDouble.of(1.0 / INCHES_PER_TICK));
            c.xPodDirection.set(X_DIRECTION < 0 ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(Y_DIRECTION < 0 ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        });
    }

    public PinpointOdometry(HardwareMap map) {
        // The robot must be stationary during the constructor's IMU calibration.
        localizer = new PinpointLocalizer(map, createConfig());
        pinpoint = map.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    /** Reset the pose without recalibrating the IMU. */
    public void reset() {
        localizer.setPose(Pose.zero());
        localizer.update();
        lastX = pinpoint.getEncoderX();
        lastY = pinpoint.getEncoderY();
        xTicks = yTicks = 0;
        lastHeading = localizer.pose().heading();
        heading = 0;
        initialized = true;
    }

    public void update() {
        if (!initialized) throw new IllegalStateException("Call reset before update");
        localizer.update();
        int rawX = pinpoint.getEncoderX(), rawY = pinpoint.getEncoderY();
        xTicks += (long) (rawX - lastX) * X_DIRECTION;
        yTicks += (long) (rawY - lastY) * Y_DIRECTION;
        lastX = rawX;
        lastY = rawY;
        double angle = localizer.pose().heading();
        heading += Math.atan2(Math.sin(angle - lastHeading), Math.cos(angle - lastHeading));
        lastHeading = angle;
    }

    public long getXTicks() { return xTicks; }
    public long getYTicks() { return yTicks; }
    public double getX() { return localizer.pose().x(); }
    public double getY() { return localizer.pose().y(); }
    public double getHeading() { return heading; }
    public GoBildaPinpointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }
}
