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

    /**
     * Displays all telemetry information
     */
    public void displayAll(SorterControls sorterControls, DrivebaseControl drivebase,
                           ShooterControls shooter, ShooterInterpolation interpolation,
                           LimelightCentering limelight, boolean centeringActive,
                           double rightTarget, double middleTarget, double leftTarget) {

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
            telemetry.addData("Distance (inches)", "%.1f", limelight.getDistanceInches());
            telemetry.addData("Interpolated RPM", "%.0f", interpolation.getInterpolatedTargetRPM());
            telemetry.addData("Servos Lowered", interpolation.areServosLowered());
        }

        // Intake and sorter
        telemetry.addLine();
        telemetry.addData("Sort Enabled", sorterControls.isSorterEnabled());
        telemetry.addData("Sort Power", sorterControls.getSorterPower());
        telemetry.addData("Sort2 Power", sorterControls.getSorter2Power());
        telemetry.addData("Intake Power", sorterControls.getIntakePower());

        // Limelight info
        telemetry.addLine();
        if (centeringActive) {
            telemetry.addData("ROTATION LOCK", limelight.isTagVisible() ? "ACTIVE" : "NO TAG");
            if (limelight.isTagVisible()) {
                telemetry.addData("Tag ID", limelight.getTagId());
                telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
                telemetry.addData("Rotation Cmd", "%.3f", -(RobotConstants.limelight_kP * limelight.getTx()));
            }
        } else if (limelight.isTagVisible()) {
            telemetry.addData("Tag ID", limelight.getTagId());
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
        } else {
            telemetry.addLine("No AprilTag");
        }

        telemetry.update();
    }
}