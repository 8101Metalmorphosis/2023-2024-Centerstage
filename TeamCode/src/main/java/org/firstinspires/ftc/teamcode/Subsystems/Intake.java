package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    public DcMotorEx intakeMotor;
    public Servo door;
    public Servo intakePivot;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        door = hardwareMap.get(Servo.class, "door");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");


    }

    public void setSpeed(float speed) {
        intakeMotor.setPower(speed);
    }

    public void pivotDown () {
        intakePivot.setPosition(Constants.IntakeConstants.intakePivotDown);
    }

    public void pivotUp() {
        intakePivot.setPosition(Constants.IntakeConstants.intakePivotReset);
    }

    public void openDoor(){
        door.setPosition(Constants.IntakeConstants.doorOpen);
    }

    public void closeDoor(){
        door.setPosition(Constants.IntakeConstants.doorClose);
    }



}
