package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Centralized telemetry display for TeleOp
 */
public class TelemetryHelper {

    private Telemetry telemetry;

    public TelemetryHelper(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private void displayCommon(SorterControls sorterControls, DrivebaseControl drivebase,
                               ShooterControls shooter, ShooterInterpolation interpolation,
                               AprilTagCentering aprilTagCentering,
                               double rightTarget, double middleTarget, double leftTarget) {
        telemetry.clearAll();

        // Color sensor data
        int[] leftRGB = sorterControls.getLeftColorRGB();
        int[] rightRGB = sorterControls.getRightColorRGB();

        telemetry.addData("LeftColor R/G/B", "%d / %d / %d", leftRGB[0], leftRGB[1], leftRGB[2]);
        telemetry.addData("RightColor R/G/B", "%d / %d / %d", rightRGB[0], rightRGB[1], rightRGB[2]);

        telemetry.addData("LeftOccupied", sorterControls.isLeftOccupied());
        telemetry.addData("RightOccupied", sorterControls.isRightOccupied());

        // Driving info
        telemetry.addLine();
        telemetry.addData("Slow Mode", drivebase.isSlowModeEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Max Power", drivebase.getMaxPower());

        // Shooter info
        telemetry.addLine();
        telemetry.addData("Shooter Enabled (L/M/R)", "%b / %b / %b",
                shooter.isLeftShooterEnabled(), shooter.isMiddleShooterEnabled(), shooter.isRightShooterEnabled());
        telemetry.addData("Shooter RPMs (L/M/R)", "%.0f / %.0f / %.0f",
                shooter.getLeftRPM(), shooter.getMiddleRPM(), shooter.getRightRPM());
        telemetry.addData("Target RPMs (L/M/R)", "%.0f / %.0f / %.0f",
                leftTarget, middleTarget, rightTarget);

        boolean allAtTarget = shooter.areAllEnabledShootersAtTarget(rightTarget, middleTarget, leftTarget);
        telemetry.addData("At Target RPM", (allAtTarget && shooter.isAnyShooterEnabled()) ? "YES" : "NO");

        // Interpolation mode
        telemetry.addLine();
        if (interpolation.isInterpolationShooting()) {
            telemetry.addData("INTERPOLATION MODE", "ACTIVE");
            telemetry.addData("Distance (inches)", "%.1f", aprilTagCentering.getDistanceInches());
            telemetry.addData("Tag Visible", aprilTagCentering.isTagVisible() ? "YES" : "NO");
            telemetry.addData("Interpolated RPM", "%.0f", interpolation.getInterpolatedTargetRPM());
            telemetry.addData("Servos Lowered", interpolation.areServosLowered());
        }

        // Intake and sorter
        telemetry.addLine();
        telemetry.addData("Sort Enabled", sorterControls.isSorterEnabled());
        telemetry.addData("Sort Power", sorterControls.getSorterPower());
        telemetry.addData("Sort2 Power", sorterControls.getSorter2Power());
        telemetry.addData("Intake Power", sorterControls.getIntakePower());
    }

    /**
     * Overload for TeleOp with AutoCenteringController (legacy compatibility)
     */
    public void displayAll(SorterControls sorterControls, DrivebaseControl drivebase,
                           ShooterControls shooter, ShooterInterpolation interpolation,
                           AprilTagCentering aprilTagCentering, AutoCenteringController autoCentering,
                           PoseEstimator poseEstimator, boolean centeringActive,
                           double rightTarget, double middleTarget, double leftTarget) {

        displayCommon(sorterControls, drivebase, shooter, interpolation, aprilTagCentering, rightTarget, middleTarget, leftTarget);

        telemetry.addLine("=== HEADING/CENTERING ===");
        if (centeringActive) {
            telemetry.addData("Mode", "AUTO-CENTERING");
           // telemetry.addData("Target Goal", autoCentering.getTargetGoalId());
            telemetry.addData("Heading Error", "%.1f deg", autoCentering.getHeadingErrorDegrees());
            telemetry.addData("Rotation Power", "%.3f", autoCentering.getRotationCorrection());
            telemetry.addData("Centered", autoCentering.isCentered() ? "YES" : "NO");
        } else {
            telemetry.addData("Mode", "MANUAL");
            telemetry.addData("Has Valid Tracking", poseEstimator.hasValidTracking() ? "YES" : "NO");
        //    telemetry.addData("Facing Goal", poseEstimator.getFacingGoal());
        }

        telemetry.update();
    }

    /**
     * Main overload for TeleOp with AprilTagCentering
     */
    public void displayAll(SorterControls sorterControls, DrivebaseControl drivebase,
                           ShooterControls shooter, ShooterInterpolation interpolation,
                           AprilTagCentering aprilTagCentering1, AprilTagCentering aprilTagCentering2,
                           boolean centeringActive,
                           double rightTarget, double middleTarget, double leftTarget) {

        displayCommon(sorterControls, drivebase, shooter, interpolation, aprilTagCentering1, rightTarget, middleTarget, leftTarget);

        telemetry.addLine("=== APRILTAG CENTERING ===");
        if (centeringActive) {
            telemetry.addData("Mode", "ACTIVE");
            telemetry.addData("Tag Visible", aprilTagCentering1.isTagVisible() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f inches", aprilTagCentering1.getDistanceInches());
            telemetry.addData("Centered", aprilTagCentering1.isCentered() ? "YES" : "NO");
        } else {
            telemetry.addData("Mode", "INACTIVE");
            telemetry.addData("Tag Visible", aprilTagCentering1.isTagVisible() ? "YES" : "NO");
        }

        telemetry.update();
    }
}