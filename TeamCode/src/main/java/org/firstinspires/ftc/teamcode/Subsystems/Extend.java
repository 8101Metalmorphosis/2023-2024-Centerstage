package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;


public class Extend {
    public DcMotorEx extend;
    public DcMotorEx intake;

    public Servo extendArm;
    public Servo intakeDoor;


    public int currentExtendPosition;
    public int targetExtendPosition;

    public double currentArmPosition;
    public double currentIntakeDoorPosition;

    public Extend(HardwareMap hardwareMap) {
        extend = hardwareMap.get(DcMotorEx.class, "leftExtend");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        extendArm = hardwareMap.get(Servo.class, "IntakePivot");
        intakeDoor = hardwareMap.get(Servo.class, "IntakeDoor");
    }

    public void update() {
        currentExtendPosition = extend.getCurrentPosition();

        currentArmPosition = extendArm.getPosition();
        currentIntakeDoorPosition = intakeDoor.getPosition();
    }

    public void setExtendPosition(int ticks) {
        targetExtendPosition = ticks;
        extend.setTargetPosition(ticks);
    }

    public void setArmPosition(float armPosition) {
        extendArm.setPosition(armPosition);
    }

    public void pivotArmDown() {
        extendArm.setPosition(Constants.ExtendConstants.intakeExtendArm);
    }

    public void pivotArmUp() {
        extendArm.setPosition(Constants.ExtendConstants.resetExtendArm);
    }


    // Intake
    public void setIntakeSpeed(float speed) {
        intake.setPower(speed);
    }

    public void openIntakeDoor() {
        intakeDoor.setPosition(Constants.IntakeConstants.doorOpen);
    }

    public void closeIntakeDoor() {
        intakeDoor.setPosition(Constants.IntakeConstants.doorClose);
    }

}
