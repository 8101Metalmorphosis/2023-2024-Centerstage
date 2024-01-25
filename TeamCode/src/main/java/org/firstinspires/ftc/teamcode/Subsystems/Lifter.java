package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Lifter {

    public DcMotorEx rightLift;

    public Servo leftArm;
    public Servo rightArm;

    public Servo WristPitch;
    public Servo WristRoll;
    public Servo claw;

    public int currentLiftPosition;
    public int targetLiftPosition;

    public double currentArmPosition;
    public double currentWristPitchPosition;
    public double currentWristRollPosition;
    public double currentClawPosition;


    public Lifter(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftArm = hardwareMap.get(Servo.class, "liftLeftArm");
        rightArm = hardwareMap.get(Servo.class, "liftRightArm");

        WristPitch = hardwareMap.get(Servo.class, "Pitch Wrist");
        WristRoll = hardwareMap.get(Servo.class, "Roll Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");

        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        currentLiftPosition = rightLift.getCurrentPosition();

        currentArmPosition = leftArm.getPosition();

        currentWristPitchPosition = WristPitch.getPosition();
        currentWristRollPosition = WristRoll.getPosition();

        currentClawPosition = claw.getPosition();
    }

    public void setLifterPosition(int ticks) {
        targetLiftPosition = ticks;

        rightLift.setTargetPosition(ticks);
    }

    public void setArmPosition(float armPosition) {
        leftArm.setPosition(armPosition);
        rightArm.setPosition(armPosition);
    }

    public void setWristPitchPosition(float wristPosition) {
        WristPitch.setPosition(wristPosition);
    }

    public void setWristRollPosition(float wristPosition) {
        WristRoll.setPosition(wristPosition);
    }


    public void clawOpen() {
        claw.setPosition(Constants.ClawConstants.clawOpen);
    }

    public void clawClose() {
        claw.setPosition(Constants.ClawConstants.clawClose);
    }
}