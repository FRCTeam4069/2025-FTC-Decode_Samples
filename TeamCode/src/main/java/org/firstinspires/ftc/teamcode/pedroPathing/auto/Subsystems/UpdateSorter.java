package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;

public class UpdateSorter {

    private CRServo sorterServo;
    private CRServo sorterServo2;

    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;

    public boolean leftOccupiedGreen = false;
    public boolean rightOccupiedGreen = false;
    public boolean leftOccupiedPurple = false;
    public boolean rightOccupiedPurple = false;

    public UpdateSorter(CRServo sorter1, CRServo sorter2,
                           RevColorSensorV3 leftColorSensor,
                           RevColorSensorV3 rightColorSensor) {

        this.sorterServo = sorter1;
        this.sorterServo2 = sorter2;
        this.leftColor = leftColorSensor;
        this.rightColor = rightColorSensor;
    }

    public void updateSorter() {

        int l = leftColor.green();
        int r = rightColor.green();

        if (l < 100 && r < 100) {
            sorterServo.setPower(1.0);
            sorterServo2.setPower(1.0);
        } else if (r > 100 && l < 100) {
            sorterServo.setPower(-1.0);
            sorterServo2.setPower(-1.0);
        } else if (l > 100 && r < 100) {
            sorterServo.setPower(1.0);
            sorterServo2.setPower(1.0);
        } else {
            sorterServo.setPower(-1.0);
            sorterServo2.setPower(1.0);
        }
    }

    public void stopSorter() {
        sorterServo.setPower(0);
        sorterServo2.setPower(0);
    }
}
