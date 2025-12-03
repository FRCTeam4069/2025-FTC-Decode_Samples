package org.firstinspires.ftc.teamcode.pedroPathing.tele.TeleSubsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the pusher/kicker servos (left and right)
 */
public class PusherServos {

    private Servo leftPushServo;
    private Servo rightPushServo;

    public PusherServos(Servo leftPushServo, Servo rightPushServo) {
        this.leftPushServo = leftPushServo;
        this.rightPushServo = rightPushServo;

        // Reverse the rightPushServo direction
        rightPushServo.setDirection(Servo.Direction.REVERSE);

        // Set startup positions (all up)
        setAllUp();
    }

    /**
     * Sets all pusher servos to up position
     */
    public void setAllUp() {
        leftPushServo.setPosition(RobotConstants.SERVO_MAX);
        rightPushServo.setPosition(RobotConstants.SERVO_MAX);
    }

    /**
     * Sets all pusher servos to down position
     */
    public void setAllDown() {
        leftPushServo.setPosition(RobotConstants.SERVO_MIN);
        rightPushServo.setPosition(RobotConstants.SERVO_MIN);
    }

    /**
     * Sets individual pusher servo positions
     */
    public void setLeft(double position) {
        leftPushServo.setPosition(position);
    }

    public void setRight(double position) {
        rightPushServo.setPosition(position);
    }
}