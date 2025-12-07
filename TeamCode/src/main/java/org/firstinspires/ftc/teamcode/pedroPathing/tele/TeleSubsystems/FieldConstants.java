package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

/**
 * Field constants for FTC DECODE 2025 season
 * Contains field dimensions and AprilTag positions
 */
public class FieldConstants {

    // Field dimensions (in inches)
    public static final double FIELD_WIDTH = 144.0;  // 144 inches
    public static final double FIELD_LENGTH = 144.0; // 144 inches (square field)

    // AprilTag size (in mm, as specified for 2025 season)
    public static final double APRILTAG_SIZE_MM = 165.1;

    /**
     * GOAL CENTER POSITIONS - These are what you want to AIM AT!
     * Goals are on the back walls, perpendicular to the field
     */
    public static final double RED_GOAL_CENTER_X = -72.0;   // inches (back wall)
    public static final double RED_GOAL_CENTER_Y = 0.0;     // inches (centered on wall)

    public static final double BLUE_GOAL_CENTER_X = 72.0;   // inches (back wall)
    public static final double BLUE_GOAL_CENTER_Y = 0.0;    // inches (centered on wall)

    /**
     * AprilTag 20 Position (Red Goal)
     * From Limelight fmap: transform position [-1.482, -1.413, 0.749] meters
     * Converting to FTC coordinate system (inches):
     * X (forward): -1.482m = -58.35 inches (back of field)
     * Y (left): -1.413m = -55.63 inches (right side when facing red goal)
     * Z (up): 0.749m = 29.49 inches (height above floor)
     *
     * NOTE: The AprilTag itself is at an ANGLE (approximately 40° from perpendicular)
     * But for shooting, we want to aim at the GOAL CENTER, not the tag!
     */
    public static final double TAG_20_X = -58.35;     // inches
    public static final double TAG_20_Y = -55.63;     // inches
    public static final double TAG_20_Z = 29.49;      // inches

    /**
     * AprilTag 20 Heading
     * From transform matrix: rotation indicates tag faces approximately 144° (SE direction)
     * This is the direction the TAG faces, not the direction you should aim!
     */
    public static final double TAG_20_HEADING = Math.toRadians(144.0); // radians

    /**
     * AprilTag 24 Position (Blue Goal)
     * From Limelight fmap: transform position [-1.482, 1.413, 0.749] meters
     * Converting to FTC coordinate system (inches):
     * X (forward): -1.482m = -58.35 inches (back of field)
     * Y (left): 1.413m = 55.63 inches (left side when facing blue goal)
     * Z (up): 0.749m = 29.49 inches (height above floor)
     *
     * NOTE: The AprilTag itself is at an ANGLE (approximately 40° from perpendicular)
     * But for shooting, we want to aim at the GOAL CENTER, not the tag!
     */
    public static final double TAG_24_X = -58.35;     // inches
    public static final double TAG_24_Y = 55.63;      // inches
    public static final double TAG_24_Z = 29.49;      // inches

    /**
     * AprilTag 24 Heading
     * From transform matrix: rotation indicates tag faces approximately 36° (NE direction)
     * This is the direction the TAG faces, not the direction you should aim!
     */
    public static final double TAG_24_HEADING = Math.toRadians(36.0); // radians

    /**
     * Camera mounting offset from robot center
     * IMPORTANT: Teams must measure and set these values for their specific robot!
     * Positive X = forward, Positive Y = left, Positive Z = up
     */
    public static final double CAMERA_OFFSET_X = -7.0;  // inches (TODO: MEASURE)
    public static final double CAMERA_OFFSET_Y = 0.0;  // inches (TODO: MEASURE)
    public static final double CAMERA_OFFSET_Z = 3.0;  // inches (TODO: MEASURE)

    /**
     * Pose estimation tuning parameters
     */
    public static final double VISION_TRUST_FACTOR = 0.7;  // 70% vision, 30% odometry
    public static final double MAX_VISION_DISTANCE = 120.0; // Don't trust vision beyond 120 inches
    public static final double MAX_POSE_AMBIGUITY = 0.2;    // Limelight pose ambiguity threshold

    /**
     * Auto-centering parameters
     * IMPORTANT: We aim at the GOAL CENTER (perpendicular to wall), NOT the AprilTag angle!
     */
    public static final double HEADING_KP = 0.0001;              // Proportional gain for heading control
    public static final double HEADING_KI = 0.0;              // Integral gain (disabled for now)
    public static final double HEADING_KD = 0.15;             // Derivative gain
    public static final double HEADING_TOLERANCE = Math.toRadians(2.0); // ±2 degrees
    public static final double MAX_ANGULAR_VELOCITY = 0.01;    // radians per second

    /**
     * Get the goal center position for the specified AprilTag ID
     * Returns the shooting target position (center of goal, perpendicular to wall)
     * NOT the position of the angled AprilTag!
     */
    public static double[] getGoalCenterForTag(int tagId) {
        if (tagId == 20) {
            return new double[]{RED_GOAL_CENTER_X, RED_GOAL_CENTER_Y};
        } else if (tagId == 24) {
            return new double[]{BLUE_GOAL_CENTER_X, BLUE_GOAL_CENTER_Y};
        }
        return new double[]{0.0, 0.0}; // Default to center if unknown
    }

    /**
     * Get AprilTag field position for specified tag ID
     * This is the tag's actual position (at an angle), used for localization
     */
    public static double[] getTagPosition(int tagId) {
        if (tagId == 20) {
            return new double[]{TAG_20_X, TAG_20_Y, TAG_20_Z};
        } else if (tagId == 24) {
            return new double[]{TAG_24_X, TAG_24_Y, TAG_24_Z};
        }
        return null;
    }

    /**
     * Get AprilTag heading for specified tag ID
     * This is the direction the TAG faces (at an angle), used for localization
     * NOT used for aiming - we aim at goal center instead!
     */
    public static Double getTagHeading(int tagId) {
        if (tagId == 20) {
            return TAG_20_HEADING;
        } else if (tagId == 24) {
            return TAG_24_HEADING;
        }
        return null;
    }
}