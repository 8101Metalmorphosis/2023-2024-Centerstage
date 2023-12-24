package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FTCutil.PID.PIDController;
import org.firstinspires.ftc.teamcode.FTCutil.PID.ProfiledPIDController;

public class Lifter {

    public DcMotorEx rightLift, leftLift;
    public int rightPos;
    public int targetPos;


    public Lifter(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");


        resetArm();
        rightPos = rightLift.getCurrentPosition();
    }

    public void zeroArm() {
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setPower(-.1);
        leftLift.setPower(-.1);
    }

    public void resetArm() {
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setTargetPosition(rightLift.getCurrentPosition());
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setTargetPosition(leftLift.getCurrentPosition());
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(.3);
        leftLift.setPower(.3);
    }

    public void setPosition(int ticks) {
        rightLift.setTargetPosition(ticks);
        leftLift.setTargetPosition(ticks);
    }

    public void update() {
        leftLift.setPower(-getPower());
        rightPos = rightLift.getCurrentPosition();
    }

    public double getPower() {
        return rightLift.getPower();
    }
}