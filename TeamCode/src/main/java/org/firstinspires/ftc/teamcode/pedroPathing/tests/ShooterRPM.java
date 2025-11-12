package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@TeleOp(name = "Shooter RPM Test", group = "Test")
public class ShooterRPM extends OpMode {

    public static double targetRPM = 3000;
    public static double kP = 0.0008;
    public static double kI = 0.0000004;
    public static double kD = 0.0002;

    DcMotor shooterMotorMiddle;

    private double integral = 0;
    private double lastError = 0;

    private int lastPosition = 0;
    private long lastTime = 0;

    private TelemetryManager tm;

    @Override
    public void init() {
        tm = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotorMiddle = hardwareMap.get(DcMotor.class, "shooterMotor");

        shooterMotorMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorMiddle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastPosition = shooterMotorMiddle.getCurrentPosition();
        lastTime = System.nanoTime();

        telemetry.addLine("✅ Shooter RPM Test Loaded");
        telemetry.addLine("Using built-in REV motor encoder");
        telemetry.addLine("Change targetRPM in Dashboard");
        telemetry.update();
    }

    private double getRPM() {
        int ticksPerRev = 28 * 19; // 28 ticks per motor rev × 19:1 gearbox
        int currentPosition = shooterMotorMiddle.getCurrentPosition();
        long currentTime = System.nanoTime();

        double deltaTicks = currentPosition - lastPosition;
        double deltaTime = (currentTime - lastTime) / 1e9;

        lastPosition = currentPosition;
        lastTime = currentTime;

        if (deltaTime <= 0) return 0;

        double ticksPerSecond = deltaTicks / deltaTime;
        return (ticksPerSecond / ticksPerRev) * 60.0; // convert to RPM
    }

    private void runShooterPID() {
        double currentRPM = getRPM();
        double error = targetRPM - currentRPM;

        integral += error;
        double derivative = error - lastError;

        double output = kP * error + kI * integral + kD * derivative;
        lastError = error;

        // Clamp motor power between 0 and 1
        output = Math.max(0, Math.min(output, 1));
        shooterMotorMiddle.setPower(output);

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Motor Power", output);
        telemetry.update();
    }

    @Override
    public void loop() {
        runShooterPID();
        tm.update();
    }

    @Override
    public void stop() {
        shooterMotorMiddle.setPower(0);
    }
}
