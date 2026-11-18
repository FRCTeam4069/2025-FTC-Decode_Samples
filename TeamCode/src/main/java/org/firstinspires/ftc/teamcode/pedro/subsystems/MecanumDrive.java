package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Robot-centric drive: forward, left and counterclockwise are positive. */
public class MecanumDrive {
    public static final String FRONT_LEFT = "lf.motor", FRONT_RIGHT = "rf.motor";
    public static final String BACK_LEFT = "lr.motor", BACK_RIGHT = "rr.motor";
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotor fl, fr, bl, br;

    public MecanumDrive(HardwareMap map) {
        fl = map.get(DcMotor.class, FRONT_LEFT);
        fr = map.get(DcMotor.class, FRONT_RIGHT);
        bl = map.get(DcMotor.class, BACK_LEFT);
        br = map.get(DcMotor.class, BACK_RIGHT);
        fl.setDirection(FRONT_LEFT_DIRECTION);
        bl.setDirection(BACK_LEFT_DIRECTION);
        fr.setDirection(FRONT_RIGHT_DIRECTION);
        br.setDirection(BACK_RIGHT_DIRECTION);
        for (DcMotor motor : new DcMotor[]{fl, fr, bl, br}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    public void drive(double forward, double left, double turn, double speed) {
        double scale = Math.max(1.0, Math.abs(forward) + Math.abs(left) + Math.abs(turn));
        fl.setPower(speed * (forward - left - turn) / scale);
        fr.setPower(speed * (forward + left + turn) / scale);
        // The robot's strafe and turn responses are swapped relative to the
        // standard rear-wheel mix; correct them before driving the motors.
        bl.setPower(speed * (forward + turn - left) / scale);
        br.setPower(speed * (forward - turn + left) / scale);
    }

    /** Run at most one wheel, using the same motor directions as the TeleOp. */
    public void testWheel(int wheel, double power) {
        fl.setPower(wheel == 0 ? power : 0);
        fr.setPower(wheel == 1 ? power : 0);
        bl.setPower(wheel == 2 ? power : 0);
        br.setPower(wheel == 3 ? power : 0);
    }

    public void stop() { drive(0, 0, 0, 1); }
}
