package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Two flywheels and feeder, using the current TeleOp's PIDF and directions. */
public class Shooter {
    public static final double MAX_RPM = 6000;
    private final DcMotorEx left, right, feeder;
    private final VoltageSensor battery;
    private final PIDF leftPid = new PIDF(), rightPid = new PIDF(), feederPid = new PIDF();
    private double leftRpm, rightRpm, feederRpm, leftPower, rightPower, feederPower, target;
    private boolean enabled;

    public Shooter(HardwareMap map) {
        left = map.get(DcMotorEx.class, "leftShooter");
        right = map.get(DcMotorEx.class, "rightShooter");
        feeder = map.get(DcMotorEx.class, "feeder");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{left, right, feeder}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        battery = map.voltageSensor.iterator().next();
        stop();
    }
    public void update(boolean on, double requestedRpm, double dt) {
        enabled = on;
        target = on ? Range.clip(requestedRpm, 0, MAX_RPM) : 0;
        dt = dt > 0 ? dt : 1e-3;
        leftRpm = left.getVelocity()/28.0*60.0;
        rightRpm = right.getVelocity()/28.0*60.0;
        feederRpm = feeder.getVelocity()/537.7*60.0;
        double voltage = battery.getVoltage();
        double scale = voltage > 1 ? 12.0/voltage : 1.0;
        if (target > 0) {
            leftPower = Range.clip(leftPid.update(target,leftRpm,0.002,0,0,0.0004*scale,0.3,dt),0,1);
            rightPower = Range.clip(rightPid.update(target,rightRpm,0.002,0,0,0.0004*scale,0.3,dt),0,1);
        } else {
            leftPid.reset(); rightPid.reset(); leftPower = rightPower = 0;
        }
        if (on) {
            feederPower = Range.clip(feederPid.update(500,feederRpm,0.002,0,0,0.0004*scale,0.3,dt),-1,1);
        } else { feederPid.reset(); feederPower = 0; }
        left.setPower(leftPower); right.setPower(rightPower); feeder.setPower(feederPower);
    }
    public void addTelemetry(Telemetry telemetry, double requestedRpm) {
        telemetry.addData("Shooter", enabled ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", requestedRpm);
        telemetry.addData("Left RPM", "%.0f (err %.0f)", leftRpm, target-leftRpm);
        telemetry.addData("Right RPM", "%.0f (err %.0f)", rightRpm, target-rightRpm);
        telemetry.addData("Left / Right power", "%.2f / %.2f", leftPower, rightPower);
        telemetry.addData("Feeder", enabled ? "ON" : "OFF");
        telemetry.addData("Feeder RPM", "%.0f / %.0f", feederRpm, enabled ? 500.0 : 0.0);
        telemetry.addData("Feeder power", "%.2f", feederPower);
        telemetry.addData("Battery V", "%.2f", battery.getVoltage());
    }
    public void stop() {
        left.setPower(0); right.setPower(0); feeder.setPower(0);
        leftPid.reset(); rightPid.reset(); feederPid.reset();
    }
    private static class PIDF {
        private double integral, lastError;
        private boolean first = true;
        void reset() { integral = 0; lastError = 0; first = true; }
        double update(double target, double measured, double kp, double ki, double kd,
                      double kf, double iMax, double dt) {
            double error = target - measured;
            integral += error * dt;
            integral = ki != 0 ? Range.clip(integral, -iMax/Math.abs(ki), iMax/Math.abs(ki)) : 0;
            double derivative = !first && dt > 0 ? (error-lastError)/dt : 0;
            first = false; lastError = error;
            return kp*error + ki*integral + kd*derivative + kf*target;
        }
    }
}
