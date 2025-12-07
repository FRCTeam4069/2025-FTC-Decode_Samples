package org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants;

public final class BlueFourCyclesConstants {

    private BlueFourCyclesConstants() {}

    // -----------------------------------------------------------------------
    // Shooter PIDF Constants
    // -----------------------------------------------------------------------
    public static final double SHOOTER_KP = 0.00015;   // Increased for better tracking
    public static final double SHOOTER_KI = 0.0000;  // Small I term to eliminate steady-state error
    public static final double SHOOTER_KD = 0.0001;  // Increased to reduce overshoot
    public static final double SHOOTER_KF = 0.0002;  // Reduced - was causing overshoot

    // Target shooter speed
    public static final double TARGET_RPM = 3150.0;   // Your actual target

    // Threshold to ensure consistent shots (95% of target)
    public static final double RPM_THRESHOLD = 3087.0;

    // Encoder constants
    public static final double TICKS_PER_REV = 28.0;

    // -----------------------------------------------------------------------
    // Servo Positions
    // -----------------------------------------------------------------------
    public static final double SERVO_MAX = 1.0;   // Up/retracted
    public static final double SERVO_MIN = 0.0;   // Down/extended

    // Delay between lowering blockers and activating push servos
    public static final double PUSH_SERVO_DELAY = 0.25;

    // -----------------------------------------------------------------------
    // Shooter Timing
    // -----------------------------------------------------------------------
    // Stagger motor startup by 50ms each to reduce voltage spike
    public static final double SHOOTER_START_DELAY = 0.05;

    // Maximum time to wait for shooters to reach speed before forcing shot
    public static final double SHOOTER_TIMEOUT = 3.0;
}