package org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants;

public final class BlueFourCyclesConstants {

    // Prevent instantiation
    private BlueFourCyclesConstants() {}

    // -----------------------------------------------------------------------
    // Shooter PIDF Constants
    // -----------------------------------------------------------------------
    public static final double SHOOTER_KP = 0.0002;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0001;
    public static final double SHOOTER_KF = 0.0004;

    // Target shooter speed
    public static final double TARGET_RPM = 3000.0;
    public static final double RPM_THRESHOLD = 2800.0;

    // Encoder constants
    public static final double TICKS_PER_REV = 28.0;

    // -----------------------------------------------------------------------
    // Servo Positions
    // -----------------------------------------------------------------------
    public static final double SERVO_MAX = 1.0;   // Up
    public static final double SERVO_MIN = 0.0;   // Down

    // Push servo delay
    public static final double PUSH_SERVO_DELAY = 0.5;

    // -----------------------------------------------------------------------
    // Shooter Timing
    // -----------------------------------------------------------------------
    public static final double SHOOTER_START_DELAY = 0.05; // 50 ms
    public static final double SHOOTER_TIMEOUT = 4.0;      // seconds
}
