package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Fuses odometry, IMU, and AprilTag vision data to maintain continuous robot pose estimate
 */
public class PoseEstimator {

    private final Follower follower;
    private final BNO055IMU imu;
    private final Limelight3A limelight;

    // Current estimated pose
    private double estimatedX = 0.0;  // inches
    private double estimatedY = 0.0;  // inches
    private double estimatedHeading = 0.0;  // radians

    // Vision state tracking
    private boolean visionTrusted = false;
    private int lastSeenTagId = -1;
    private long lastVisionUpdateTime = 0;
    private double lastVisionX = 0.0;
    private double lastVisionY = 0.0;

    // Odometry tracking
    private double lastOdomX = 0.0;
    private double lastOdomY = 0.0;
    private double lastOdomHeading = 0.0;

    public PoseEstimator(Follower follower, BNO055IMU imu, Limelight3A limelight) {
        this.follower = follower;
        this.imu = imu;
        this.limelight = limelight;

        // Initialize from follower's starting pose
        this.lastOdomX = follower.getPose().getX();
        this.lastOdomY = follower.getPose().getY();
        this.lastOdomHeading = follower.getPose().getHeading();

        this.estimatedX = lastOdomX;
        this.estimatedY = lastOdomY;
        this.estimatedHeading = lastOdomHeading;
    }

    /**
     * Main update loop - call this every cycle
     */
    public void update() {
        // 1. Update from odometry (always runs)
        updateFromOdometry();

        // 2. Update from IMU heading (always runs)
        updateFromIMU();

        // 3. Try to update from AprilTag vision (when available)
        updateFromVision();
    }

    /**
     * Update pose estimate based on odometry deltas
     */
    private void updateFromOdometry() {
        double currentOdomX = follower.getPose().getX();
        double currentOdomY = follower.getPose().getY();

        // Calculate deltas from last update
        double deltaX = currentOdomX - lastOdomX;
        double deltaY = currentOdomY - lastOdomY;

        // Apply deltas to estimated pose
        estimatedX += deltaX;
        estimatedY += deltaY;

        // Store for next cycle
        lastOdomX = currentOdomX;
        lastOdomY = currentOdomY;
    }

    /**
     * Update heading from IMU (more accurate than dead reckoning)
     */
    private void updateFromIMU() {
        // Get IMU heading directly
        double imuHeading = imu.getAngularOrientation().firstAngle; // Returns radians
        estimatedHeading = normalizeAngle(imuHeading);
    }

    /**
     * Update pose from AprilTag vision when available
     */
    private void updateFromVision() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            visionTrusted = false;
            return;
        }

        // Get fiducial (AprilTag) results
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            visionTrusted = false;
            return;
        }

        // Process the first (best) detected tag
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        int tagId = (int) fiducial.getFiducialId();

        // Only process tags 20 and 24 (goal tags)
        if (tagId != 20 && tagId != 24) {
            visionTrusted = false;
            return;
        }

        // Get 3D pose of robot relative to tag (from Limelight's BotPose)
        Pose3D botpose = result.getBotpose();

        if (botpose == null) {
            visionTrusted = false;
            return;
        }

        // Check if vision data is reliable based on distance and ambiguity
        double distance = Math.hypot(
                botpose.getPosition().x,
                botpose.getPosition().y
        );

        // Convert distance to inches (Limelight returns meters)
        distance = distance * 39.3701;

        if (distance > FieldConstants.MAX_VISION_DISTANCE) {
            visionTrusted = false;
            return;
        }

        // Calculate robot's field pose from vision
        double[] robotFieldPose = calculateRobotFieldPose(tagId, botpose);

        if (robotFieldPose == null) {
            visionTrusted = false;
            return;
        }

        double visionX = robotFieldPose[0];
        double visionY = robotFieldPose[1];
        double visionHeading = robotFieldPose[2];

        // Fuse vision with current estimate using trust factor
        double trustFactor = FieldConstants.VISION_TRUST_FACTOR;

        estimatedX = trustFactor * visionX + (1 - trustFactor) * estimatedX;
        estimatedY = trustFactor * visionY + (1 - trustFactor) * estimatedY;
        estimatedHeading = normalizeAngle(
                trustFactor * visionHeading + (1 - trustFactor) * estimatedHeading
        );

        // Update vision state
        visionTrusted = true;
        lastSeenTagId = tagId;
        lastVisionUpdateTime = System.currentTimeMillis();
        lastVisionX = visionX;
        lastVisionY = visionY;
    }

    /**
     * Calculate robot's field pose from AprilTag detection
     *
     * @param tagId The detected AprilTag ID (20 or 24)
     * @param botpose Robot pose relative to tag (from Limelight)
     * @return [fieldX, fieldY, fieldHeading] or null if invalid
     */
    private double[] calculateRobotFieldPose(int tagId, Pose3D botpose) {
        // Get tag's known field position
        double[] tagPos = FieldConstants.getTagPosition(tagId);
        Double tagHeading = FieldConstants.getTagHeading(tagId);

        if (tagPos == null || tagHeading == null) {
            return null;
        }

        double tagFieldX = tagPos[0];
        double tagFieldY = tagPos[1];

        // Get robot position relative to tag (convert from meters to inches)
        // Limelight coordinate system: X forward, Y left, Z up
        double robotRelativeX = botpose.getPosition().x * 39.3701;  // Convert to inches
        double robotRelativeY = botpose.getPosition().y * 39.3701;
        double robotRelativeZ = botpose.getPosition().z * 39.3701;

        // Get robot orientation relative to tag
        double robotRelativeYaw = Math.toRadians(botpose.getOrientation().getYaw());

        // Transform robot-relative position to field coordinates
        // Rotate by tag's heading, then translate by tag's position
        double cosTag = Math.cos(tagHeading);
        double sinTag = Math.sin(tagHeading);

        // Apply rotation
        double rotatedX = robotRelativeX * cosTag - robotRelativeY * sinTag;
        double rotatedY = robotRelativeX * sinTag + robotRelativeY * cosTag;

        // Translate by tag position
        double robotFieldX = tagFieldX + rotatedX;
        double robotFieldY = tagFieldY + rotatedY;

        // Calculate robot's field heading
        double robotFieldHeading = normalizeAngle(tagHeading + robotRelativeYaw);

        return new double[]{robotFieldX, robotFieldY, robotFieldHeading};
    }

    /**
     * Reset the pose estimate to a specific position
     */
    public void resetPose(double x, double y, double heading) {
        this.estimatedX = x;
        this.estimatedY = y;
        this.estimatedHeading = normalizeAngle(heading);

        // Also reset odometry tracking
        this.lastOdomX = follower.getPose().getX();
        this.lastOdomY = follower.getPose().getY();
        this.lastOdomHeading = follower.getPose().getHeading();
    }

    /**
     * Get current estimated X position (inches)
     */
    public double getX() {
        return estimatedX;
    }

    /**
     * Get current estimated Y position (inches)
     */
    public double getY() {
        return estimatedY;
    }

    /**
     * Get current estimated heading (radians)
     */
    public double getHeading() {
        return estimatedHeading;
    }

    /**
     * Get current estimated heading (degrees)
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(estimatedHeading);
    }

    /**
     * Check if vision is currently trusted
     */
    public boolean isVisionTrusted() {
        // Vision is untrusted if we haven't seen a tag in 1 second
        if (System.currentTimeMillis() - lastVisionUpdateTime > 1000) {
            return false;
        }
        return visionTrusted;
    }

    /**
     * Get the ID of the last seen AprilTag
     */
    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    /**
     * Calculate distance to a target goal (based on last seen tag)
     * Returns distance in inches
     */
    public double getDistanceToGoal() {
        if (lastSeenTagId == -1) {
            return -1.0; // Unknown
        }

        double[] goalCenter = FieldConstants.getGoalCenterForTag(lastSeenTagId);
        double goalX = goalCenter[0];
        double goalY = goalCenter[1];

        double dx = goalX - estimatedX;
        double dy = goalY - estimatedY;

        return Math.hypot(dx, dy);
    }

    /**
     * Calculate angle to target goal (based on last seen tag)
     * Returns angle in radians
     */
    public double getAngleToGoal() {
        if (lastSeenTagId == -1) {
            return 0.0; // Unknown
        }

        double[] goalCenter = FieldConstants.getGoalCenterForTag(lastSeenTagId);
        double goalX = goalCenter[0];
        double goalY = goalCenter[1];

        double dx = goalX - estimatedX;
        double dy = goalY - estimatedY;

        return Math.atan2(dy, dx);
    }

    /**
     * Get heading error to face the goal
     * Returns the angle correction needed (radians)
     * Positive = turn left, Negative = turn right
     */
    public double getHeadingErrorToGoal() {
        double targetHeading = getAngleToGoal();
        return normalizeAngle(targetHeading - estimatedHeading);
    }

    /**
     * Determine which goal is CLOSEST to the robot (by distance)
     * Returns 20 (red) or 24 (blue) based on which goal is physically closer
     */
    public int getClosestGoal() {
        // Calculate distance to red goal
        double dxRed = FieldConstants.RED_GOAL_CENTER_X - estimatedX;
        double dyRed = FieldConstants.RED_GOAL_CENTER_Y - estimatedY;
        double distanceToRed = Math.hypot(dxRed, dyRed);

        // Calculate distance to blue goal
        double dxBlue = FieldConstants.BLUE_GOAL_CENTER_X - estimatedX;
        double dyBlue = FieldConstants.BLUE_GOAL_CENTER_Y - estimatedY;
        double distanceToBlue = Math.hypot(dxBlue, dyBlue);

        // Return the closest goal
        return (distanceToRed < distanceToBlue) ? 20 : 24;
    }

    /**
     * Determine which goal we're generally facing
     * Returns 20 (red) or 24 (blue) based on which goal we're closest to facing
     */
    public int getFacingGoal() {
        // Calculate angle to each goal
        double angleToRed = Math.atan2(
                FieldConstants.RED_GOAL_CENTER_Y - estimatedY,
                FieldConstants.RED_GOAL_CENTER_X - estimatedX
        );

        double angleToBlue = Math.atan2(
                FieldConstants.BLUE_GOAL_CENTER_Y - estimatedY,
                FieldConstants.BLUE_GOAL_CENTER_X - estimatedX
        );

        // Calculate heading error to each goal
        double errorToRed = Math.abs(normalizeAngle(angleToRed - estimatedHeading));
        double errorToBlue = Math.abs(normalizeAngle(angleToBlue - estimatedHeading));

        // Return the goal we're more closely facing
        return (errorToRed < errorToBlue) ? 20 : 24;
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}