package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Lifter {

//    public DcMotorEx rightLift, leftLift;
//    public Servo rightArm;
    public Servo leftArm;
//    public int leftPos;
    public int targetPos;



    public Lifter(HardwareMap hardwareMap) {
//        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
//        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

//        rightArm = hardwareMap.get(Servo.class, "liftRightArm");
        leftArm = hardwareMap.get(Servo.class, "liftLeftArm");

        leftArm.setDirection(Servo.Direction.REVERSE);


//        resetArm();
//        leftPos = rightLift.getCurrentPosition();
    }

    public void zeroArm() {
//        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftLift.setPower(-.1);
//        resetArm();
    }

    public void resetArm() {
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftLift.setTargetPosition(0);
//        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        leftLift.setPower(Constants.LifterConstants.lifterSpeed);
//        rightLift.setPower(leftLift.getPower());
    }

    public void update() {
//        leftPos = rightLift.getCurrentPosition();
//
//        leftLift.setPower(-getPower());
    }

    public void setArmPosition(float armPosition) {
//        rightArm.setPosition(armPosition);
        leftArm.setPosition(armPosition);
    }

    public void setLifterPosition(int ticks) {
//        rightLift.setTargetPosition(ticks);
    }

    public double getPower() {
//        return rightLift.getPower();
        return 0;
    }
}