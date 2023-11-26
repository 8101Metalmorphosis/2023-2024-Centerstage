package org.firstinspires.ftc.teamcode.Subsystems;

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


    public ProfiledPIDController pidController;

    public Lifter(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");


        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

//        rightLift.setTargetPosition(rightLift.getCurrentPosition());
//        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLift.setPower(.3);
        rightPos = rightLift.getCurrentPosition();
        targetPos = rightPos;

        pidController =
                new ProfiledPIDController(2, .01, 0, Constants.Lifter.maxAccel, Constants.Lifter.maxVel, rightPos);
    }

    public void setPosition(int ticks) {
//        rightLift.setTargetPosition(ticks);
        targetPos = ticks;
    }

    public void update() {
//        leftLift.setPower(getPower());
        rightPos = rightLift.getCurrentPosition();





        double pow = pidController.update(rightPos, targetPos);

        rightLift.setVelocity(pow);
        leftLift.setVelocity(pow);
    }

    public double getPower() {
//        return rightLift.getPower();
        return pidController.update(rightPos, targetPos);
    }
}