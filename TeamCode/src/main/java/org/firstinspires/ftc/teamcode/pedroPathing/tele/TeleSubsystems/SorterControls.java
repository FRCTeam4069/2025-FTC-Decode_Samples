package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controls intake motor, sorter servos, and color sensors
 */
public class SorterControls {

    private DcMotor intake;
    private CRServo sorterServo;
    private CRServo sorterServo2;
    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;

    private boolean sorterEnabled = true;
    private boolean inwardSpinMode = false;

    public SorterControls(DcMotor intake, CRServo sorterServo, CRServo sorterServo2,
                          RevColorSensorV3 leftColor, RevColorSensorV3 rightColor) {
        this.intake = intake;
        this.sorterServo = sorterServo;
        this.sorterServo2 = sorterServo2;
        this.leftColor = leftColor;
        this.rightColor = rightColor;

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets intake motor power
     */
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    /**
     * Updates sorter servos with default spinning or manual override
     */
    public void updateSorter(double leftStickX) {
        double sorterPower1 = 0.0;
        double sorterPower2 = 0.0;

        if (sorterEnabled) {
            // Check for manual override with left stick
            if (Math.abs(leftStickX) > 0.1) { // deadband
                // Manual override: left = counter-clockwise, right = clockwise
                sorterPower1 = leftStickX;
                sorterPower2 = leftStickX;
            } else if (inwardSpinMode) {
                // Inward spin mode (toggled by X button)
                sorterPower1 = -1.0;
                sorterPower2 = 1.0;
            } else {
                // Default: both spinning in opposite directions
                sorterPower1 = 1.0;
                sorterPower2 = -1.0;
            }
        }

        sorterServo.setPower(sorterPower1);
        sorterServo2.setPower(sorterPower2);
    }

    /**
     * Toggles sorter enabled state
     */
    public void toggleSorter() {
        sorterEnabled = !sorterEnabled;
    }

    /**
     * Toggles inward spin mode
     */
    public void toggleInwardSpin() {
        inwardSpinMode = !inwardSpinMode;
    }

    /**
     * Reads left color sensor values
     */
    public int[] getLeftColorRGB() {
        return new int[]{leftColor.red(), leftColor.green(), leftColor.blue()};
    }

    /**
     * Reads right color sensor values
     */
    public int[] getRightColorRGB() {
        return new int[]{rightColor.red(), rightColor.green(), rightColor.blue()};
    }

    /**
     * Checks if left sensor detects an object
     */
    public boolean isLeftOccupied() {
        int[] rgb = getLeftColorRGB();
        return (rgb[0] > RobotConstants.SIDE_DETECT_THRESHOLD) ||
                (rgb[1] > RobotConstants.SIDE_DETECT_THRESHOLD) ||
                (rgb[2] > RobotConstants.SIDE_DETECT_THRESHOLD);
    }

    /**
     * Checks if right sensor detects an object
     */
    public boolean isRightOccupied() {
        int[] rgb = getRightColorRGB();
        return (rgb[0] > RobotConstants.SIDE_DETECT_THRESHOLD) ||
                (rgb[1] > RobotConstants.SIDE_DETECT_THRESHOLD) ||
                (rgb[2] > RobotConstants.SIDE_DETECT_THRESHOLD);
    }

    // Getters
    public boolean isSorterEnabled() {
        return sorterEnabled;
    }

    public boolean isInwardSpinMode() {
        return inwardSpinMode;
    }

    public double getSorterPower() {
        return sorterServo.getPower();
    }

    public double getSorter2Power() {
        return sorterServo2.getPower();
    }

    public double getIntakePower() {
        return intake.getPower();
    }
}