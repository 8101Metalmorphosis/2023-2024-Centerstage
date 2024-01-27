package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Lifter {

    public DcMotorEx leftLift;
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
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftArm = hardwareMap.get(Servo.class, "leftLiftArm");
        rightArm = hardwareMap.get(Servo.class, "rightLiftArm");

        WristPitch = hardwareMap.get(Servo.class, "wristPitch");
        WristRoll = hardwareMap.get(Servo.class, "wristRoll");
        claw = hardwareMap.get(Servo.class, "Claw");

        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);


        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        currentLiftPosition = rightLift.getCurrentPosition();

        currentArmPosition = leftArm.getPosition();

        currentWristPitchPosition = WristPitch.getPosition();
        currentWristRollPosition = WristRoll.getPosition();

        currentClawPosition = claw.getPosition();
    }

    public void resetArm() {
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setPower(Constants.LifterConstants.lifterSpeed);
        rightLift.setPower(Constants.LifterConstants.lifterSpeed);

        leftLift.setTargetPosition(Constants.LifterConstants.lifterMinHeight);
        rightLift.setTargetPosition(Constants.LifterConstants.lifterMinHeight);

        leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void zeroArm() {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setPower(Constants.LifterConstants.lifterZeroSpeed);
        rightLift.setPower(Constants.LifterConstants.lifterZeroSpeed);
    }

    public void setLifterPosition(int ticks) {
        targetLiftPosition = ticks;

        leftLift.setTargetPosition(ticks);
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