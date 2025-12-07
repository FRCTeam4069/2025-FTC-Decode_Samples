package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Manages Limelight AprilTag detection and rotation locking/centering
 * Corrected to use:
 *  - 2D tx for centering
 *  - 3D pose only for distance
 */
public class LimelightCentering {

    private Limelight3A limelight;
    private double lastTxError = 0.0;

    // Current tag data
    private boolean tagVisible = false;
    private int tagId = -1;
    private double tx = 0.0;
    private double distanceInches = 0.0;

    public LimelightCentering(Limelight3A limelight) {
        this.limelight = limelight;
    }

    /**
     * Updates tag detection data from Limelight
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        tagVisible = false;
        tagId = -1;
        tx = 0.0;
        distanceInches = 0.0;

        if (result == null || !result.isValid()) {
            return;
        }

        List<FiducialResult> fid = result.getFiducialResults();
        if (fid == null || fid.isEmpty()) {
            return;
        }

        // ✅ SAFELY FIND THE GOAL TAG (NOT JUST INDEX 0)
        for (FiducialResult f : fid) {
            int id = f.getFiducialId();

            // Goal tags
            if (id == 20 || id == 24) {
                tagId = id;

                Pose3D targetPose = f.getTargetPoseCameraSpace();
                if (targetPose != null) {
                    double x = targetPose.getPosition().x;
                    double y = targetPose.getPosition().y;
                    double z = targetPose.getPosition().z;

                    // ✅ TRUE 2D TX FOR CENTERING
                    tx = result.getTx();

                    // ✅ TRUE 3D DISTANCE FOR SHOOTER INTERPOLATION
                    double distanceMeters = Math.sqrt(x * x + y * y + z * z);
                    distanceInches = distanceMeters * 39.3701;

                    tagVisible = true;
                }
                break; // Stop once we find the correct tag
            }
        }
    }

    /**
     * Calculates rotation correction using PID control for centering on target
     */
    public double calculateRotationCorrection(double dt) {
        if (!tagVisible || dt <= 0) {
            return 0.0;
        }

        double error = tx;

        if (Math.abs(error) > RobotConstants.limelight_deadband) {
            double derivative = (error - lastTxError) / dt;

            double rotate =
                    -(RobotConstants.limelight_kP * error
                            + RobotConstants.limelight_kD * derivative);

            if (Math.abs(rotate) < RobotConstants.limelight_minCmd) {
                rotate = RobotConstants.limelight_minCmd * Math.signum(rotate);
            }

            rotate = Range.clip(
                    rotate,
                    -RobotConstants.limelight_maxSpeed,
                    RobotConstants.limelight_maxSpeed
            );

            lastTxError = error;
            return rotate;
        } else {
            lastTxError = error;
            return 0.0;
        }
    }

    // =======================
    // Getters
    // =======================

    public boolean isTagVisible() {
        return tagVisible;
    }

    public int getTagId() {
        return tagId;
    }

    public double getTx() {
        return tx;
    }

    public double getDistanceInches() {
        return distanceInches;
    }
}
