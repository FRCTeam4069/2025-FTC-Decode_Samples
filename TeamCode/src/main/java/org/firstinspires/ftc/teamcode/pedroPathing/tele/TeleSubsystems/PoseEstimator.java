package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

/**
 * SUPER SIMPLE heading tracker
 * Only tracks heading relative to AprilTag 20
 * Uses vision when available, ODOMETRY for heading changes when not
 * Camera is at FRONT of robot
 */
public class PoseEstimator {
    private final Follower follower;
    private final Limelight3A limelight;

    // Positive = tag is LEFT of camera  → robot must turn RIGHT to face it
    // Negative = tag is RIGHT of camera → robot must turn LEFT to face it
    private double headingErrorToTag = 0.0; // radians

    private double lastOdometryHeading = 0.0;

    private boolean visionActive = false;
    private boolean hasEverSeenTag = false;

    public PoseEstimator(Follower follower, Limelight3A limelight) {
        this.follower = follower;
        this.limelight = limelight;
        this.lastOdometryHeading = follower.getPose().getHeading();
    }

    public void update() {
        boolean gotVision = updateFromVision();
        if (!gotVision && hasEverSeenTag) {
            updateFromOdometry();
        }
    }

    private boolean updateFromVision() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            visionActive = false;
            return false;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            visionActive = false;
            return false;
        }

        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        int tagId = (int) fiducial.getFiducialId();

        if (tagId != 20) {
            visionActive = false;
            return false;
        }

        // FIXED: Limelight tx > 0 means tag is to the RIGHT → we need to turn RIGHT (negative rotation)
        double tx = fiducial.getTargetXDegrees();
        headingErrorToTag = -Math.toRadians(tx);  // ← THIS IS THE CRITICAL FIX

        lastOdometryHeading = follower.getPose().getHeading();

        visionActive = true;
        hasEverSeenTag = true;
        return true;
    }

    private void updateFromOdometry() {
        double currentOdometryHeading = follower.getPose().getHeading();
        double headingChange = normalizeAngle(currentOdometryHeading - lastOdometryHeading);

        // If robot turns right (negative change), tag appears more to the left → error becomes more positive
        headingErrorToTag = normalizeAngle(headingErrorToTag + headingChange);

        lastOdometryHeading = currentOdometryHeading;
    }

    public double getHeadingErrorToTag() {
        return headingErrorToTag;
    }

    public double getHeadingErrorDegrees() {
        return Math.toDegrees(headingErrorToTag);
    }

    public boolean hasValidTracking() {
        return hasEverSeenTag;
    }

    public boolean isVisionActive() {
        return visionActive;
    }

    public void reset() {
        headingErrorToTag = 0.0;
        hasEverSeenTag = false;
        visionActive = false;
        lastOdometryHeading = follower.getPose().getHeading();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}