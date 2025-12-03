package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

/**
 * Central configuration file for all robot constants and tunable parameters
 */
public class RobotConstants {

    // -----------------------------------------------------------------------
    // Shooter PIDF + target RPM values
    // -----------------------------------------------------------------------
    // Right shooter
    public static double right_kP = 0.00015;
    public static double right_kI = 0.0;
    public static double right_kD = 0.0001;
    public static double right_kF = 0.00025;
    public static double right_targetRPM = 3500.0;

    // Middle shooter
    public static double middle_kP = 0.00015;
    public static double middle_kI = 0.0;
    public static double middle_kD = 0.0001;
    public static double middle_kF = 0.0002;
    public static double middle_targetRPM = 3500.0;

    // Left shooter
    public static double left_kP = 0.00015;
    public static double left_kI = 0.0;
    public static double left_kD = 0.0001;
    public static double left_kF = 0.0002;
    public static double left_targetRPM = 3500.0;

    // -----------------------------------------------------------------------
    // Interpolation offset
    // -----------------------------------------------------------------------
    public static double rpm_offset = 0.0;

    // -----------------------------------------------------------------------
    // Limelight PID values
    // -----------------------------------------------------------------------
    public static double limelight_kP = 0.012;
    public static double limelight_kD = 0.002;
    public static double limelight_minCmd = 0.06;
    public static double limelight_deadband = 1.8;
    public static double limelight_maxSpeed = 0.3;

    // -----------------------------------------------------------------------
    // Shooter RPM tolerance and vibration settings
    // -----------------------------------------------------------------------
    public static double rpm_tolerance = 100.0;
    public static int vibration_duration_ms = 200;

    // -----------------------------------------------------------------------
    // Other constants
    // -----------------------------------------------------------------------
    public static final double TRIGGER_THRESHOLD = 0.3;

    // Servo positions
    public static final double SERVO_MAX = 1.0;
    public static final double SERVO_MIN = 0.0;

    // Color sensor thresholds
    public static final int SIDE_DETECT_THRESHOLD = 100;

    // Encoder specs
    public static final double TICKS_PER_REV = 28.0;

    // Push servo delay
    public static final double PUSH_SERVO_DELAY = 0.25;

    // -----------------------------------------------------------------------
    // Interpolation data points (distance in inches -> RPM)
    // -----------------------------------------------------------------------
    public static final double[] DISTANCE_DATA = {58.41, 66.48, 75.60, 85.44, 95.77, 106.45, 117.39, 128.51};
    public static final double[] RPM_DATA = {3150, 3250, 3300, 3400, 3500, 3700, 3900, 4200};
}