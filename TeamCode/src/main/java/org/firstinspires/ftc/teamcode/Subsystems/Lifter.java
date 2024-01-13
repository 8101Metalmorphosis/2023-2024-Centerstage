package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Lifter {

    public DcMotorEx rightLift;

    public Servo leftArm;
    public Servo rightArm;

    public Servo wrist;
    public Servo claw;

    public int currentLiftPosition;
    public int targetLiftPosition;

    public double currentArmPosition;
    public double currentWristPosition;
    public double currentClawPosition;


    public Lifter(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftArm = hardwareMap.get(Servo.class, "liftLeftArm");
        rightArm = hardwareMap.get(Servo.class, "liftRightArm");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");

        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        currentLiftPosition = rightLift.getCurrentPosition();

        currentArmPosition = leftArm.getPosition();
        currentWristPosition = wrist.getPosition();
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

    public void setWristPosition(float wristPosition) {
        wrist.setPosition(wristPosition);
    }



    public void clawOpen() {
        claw.setPosition(Constants.ClawConstants.clawOpen);
    }

    public void clawClose() {
        claw.setPosition(Constants.ClawConstants.clawClose);
    }


    public void zeroArm() {
        // Move arm down, in torque mode
    }

    public void resetArm() {
        // Reset encoder, turn on run to position
    }


    public double getPower() {
        return rightLift.getPower();
    }
}