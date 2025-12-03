package org.firstinspires.ftc.teamcode.pedroPathing.auto.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.auto.AutoConstants.BlueFourCyclesConstants;
import com.pedropathing.follower.Follower;

public class StopRobot {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx rightShooter, middleShooter, leftShooter;
    private CRServo sorterServo, sorterServo2;
    private Servo leftServo, middleServo, rightServo, leftPushServo, rightPushServo;

    public StopRobot(Follower follower,
                     DcMotor intake,
                     DcMotorEx rightShooter, DcMotorEx middleShooter, DcMotorEx leftShooter,
                     CRServo sorterServo, CRServo sorterServo2,
                     Servo leftServo, Servo middleServo, Servo rightServo,
                     Servo leftPushServo, Servo rightPushServo) {
        this.follower = follower;
        this.intake = intake;
        this.rightShooter = rightShooter;
        this.middleShooter = middleShooter;
        this.leftShooter = leftShooter;
        this.sorterServo = sorterServo;
        this.sorterServo2 = sorterServo2;
        this.leftServo = leftServo;
        this.middleServo = middleServo;
        this.rightServo = rightServo;
        this.leftPushServo = leftPushServo;
        this.rightPushServo = rightPushServo;
    }

    public void stopAll() {
        // Stop drivetrain
        follower.setTeleOpDrive(0, 0, 0, true);

        // Stop intake
        intake.setPower(0);

        // Stop shooters
        rightShooter.setPower(0);
        middleShooter.setPower(0);
        leftShooter.setPower(0);

        // Stop sorter
        sorterServo.setPower(0);
        sorterServo2.setPower(0);

        // Reset all servos to up position
        leftServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        middleServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        leftPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
        rightPushServo.setPosition(BlueFourCyclesConstants.SERVO_MAX);
    }
}
