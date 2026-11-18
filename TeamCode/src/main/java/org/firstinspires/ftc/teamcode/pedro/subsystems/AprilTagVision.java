package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Limelight BIOBUZZ tag tracking. The driver owns translation while aiming. */
public class AprilTagVision {
    public static final int PIPELINE = 0; // Set to the configured AprilTag pipeline.
    public static final int TARGET_ID = -1; // -1: lock the visible BIOBUZZ tag closest to straight ahead.
    public static final double CAMERA_PITCH_UP_DEGREES = 70.0;
    public static final double TARGET_BEARING_DEGREES = 0.0; // Calibrate for camera/shooter alignment.
    public static final double KP = 0.015;
    public static final double MAX_TURN = 0.25;
    public static final double TOLERANCE_DEGREES = 1.5;
    public static final long MAX_STALENESS_MS = 250;

    private final Limelight3A limelight;
    private int lockedId = -1;
    private boolean visible;
    private double errorDegrees;

    public AprilTagVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    /** Positive output means counterclockwise robot rotation. */
    public double update(boolean active) {
        visible = false;
        if (!active) { lockedId = -1; return 0; }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != PIPELINE
                || result.getStaleness() < 0
                || result.getStaleness() > MAX_STALENESS_MS) return 0;
        FiducialResult selected = null;
        double selectedError = 0;
        for (FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            double bearing = robotBearingDegrees(tag.getTargetXDegreesNoCrosshair(),
                    tag.getTargetYDegreesNoCrosshair());
            if (id < 30 || id > 45 || !Double.isFinite(bearing)) continue;
            if (TARGET_ID >= 0 && id != TARGET_ID) continue;
            if (lockedId >= 0 && id != lockedId) continue;
            double candidateError = bearing - TARGET_BEARING_DEGREES;
            if (selected == null || Math.abs(candidateError) < Math.abs(selectedError)) {
                selected = tag;
                selectedError = candidateError;
            }
        }
        if (selected == null) return 0;
        lockedId = selected.getFiducialId();
        visible = true;
        errorDegrees = selectedError;
        return turnForError(errorDegrees);
    }

    /** Robot-relative bearing from an upright camera pitched upward, positive to the right. */
    public static double robotBearingDegrees(double txDegrees, double tyDegrees) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(tyDegrees)
                || Math.abs(txDegrees) >= 89 || Math.abs(tyDegrees) >= 89) return Double.NaN;
        double pitch = Math.toRadians(CAMERA_PITCH_UP_DEGREES);
        double right = Math.tan(Math.toRadians(txDegrees));
        double up = Math.tan(Math.toRadians(tyDegrees));
        double forward = Math.cos(pitch) - Math.sin(pitch) * up;
        // Near/behind overhead the horizontal bearing is unstable or cannot be
        // reached by a small forward-facing turn. Do not command auto rotation.
        if (forward <= 0.05) return Double.NaN;
        return Math.toDegrees(Math.atan2(right, forward));
    }
    public static double turnForError(double errorDegrees) {
        if (!Double.isFinite(errorDegrees) || Math.abs(errorDegrees) <= TOLERANCE_DEGREES) return 0;
        // Limelight positive tx is to camera-right; robot CCW is positive.
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, -KP * errorDegrees));
    }
    public int getLockedId() { return lockedId; }
    public boolean isVisible() { return visible; }
    public double getErrorDegrees() { return errorDegrees; }
    public void stop() { limelight.stop(); }
}
