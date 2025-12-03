package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the three blocker servos (left, middle, right)
 */
public class BlockerServos {

    private Servo leftServo;
    private Servo middleServo;
    private Servo rightServo;

    public BlockerServos(Servo leftServo, Servo middleServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.middleServo = middleServo;
        this.rightServo = rightServo;

        // Set startup positions (all up)
        setAllUp();
    }

    /**
     * Sets all blocker servos to up position
     */
    public void setAllUp() {
        leftServo.setPosition(RobotConstants.SERVO_MAX);
        middleServo.setPosition(RobotConstants.SERVO_MAX);
        rightServo.setPosition(RobotConstants.SERVO_MAX);
    }

    /**
     * Sets all blocker servos to down position
     */
    public void setAllDown() {
        leftServo.setPosition(RobotConstants.SERVO_MIN);
        middleServo.setPosition(RobotConstants.SERVO_MIN);
        rightServo.setPosition(RobotConstants.SERVO_MIN);
    }

    /**
     * Sets individual blocker servo positions
     */
    public void setLeft(double position) {
        leftServo.setPosition(position);
    }

    public void setMiddle(double position) {
        middleServo.setPosition(position);
    }

    public void setRight(double position) {
        rightServo.setPosition(position);
    }
}