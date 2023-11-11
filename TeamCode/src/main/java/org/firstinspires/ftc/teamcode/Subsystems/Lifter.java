package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FTCutil.PID.PIDController;
import org.firstinspires.ftc.teamcode.FTCutil.PID.ProfiledPIDController;

public class Lifter {

    public DcMotorEx masterSlide, secondarySlide;
    public int masterPos;
    public int targetPos;


    public ProfiledPIDController pidController;

    public Lifter(HardwareMap hardwareMap) {
        masterSlide = hardwareMap.get(DcMotorEx.class, "masterSlide");
        secondarySlide = hardwareMap.get(DcMotorEx.class, "secondarySlide");


        masterSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        masterPos = masterSlide.getCurrentPosition();
        targetPos = masterPos;

        pidController =
                new ProfiledPIDController(1, 0, 0, Constants.Lifter.maxAccel, Constants.Lifter.maxVel, masterPos);
    }

    public void setPosition(int ticks) {
        targetPos = ticks;
    }

    public void setLength(double length) {
        setPosition((int) (length * Constants.Lifter.ticksPerInch));
    }

    public void setHeight(double height) {
        double slideLength = height / Math.sin(Math.toRadians(Constants.Lifter.angleDEG));

        setPosition((int) (slideLength * Constants.Lifter.ticksPerInch));
    }

    public void setWidth(double width) {
        double slideLength = width / Math.cos(Math.toRadians(Constants.Lifter.angleDEG));

        setPosition((int) (slideLength * Constants.Lifter.ticksPerInch));
    }

    public void update() {
        masterPos = masterSlide.getCurrentPosition();

        double pow = pidController.update(masterPos, targetPos);

        masterSlide.setVelocity(pow);
    }

    public double getPower() {
        return pidController.update(masterPos, targetPos);
    }
}