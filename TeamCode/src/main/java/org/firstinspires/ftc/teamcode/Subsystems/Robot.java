package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;

import java.util.List;

public class Robot {

    HardwareMap hardwareMap;

    // IMU
    BNO055IMU imu;

    public Orientation orientation;
    public AngularVelocity angularVelocity;

    // Motors
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;



    public Servo Claw;

    public MecanumDrive drive;
    public Lifter lifter;

    int FrontLeftPosition = 0;
    int FrontRightPosition = 0;
    int BackLeftPosition = 0;
    int BackRightPosition = 0;

    public double imuYawOffset = 0;

    double imuYaw = 0;


    List<LynxModule> allHubs;



    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drive = new MecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());


        orientation = imu.getAngularOrientation();
        angularVelocity = imu.getAngularVelocity();



        Claw = hardwareMap.get(Servo.class, "Claw");




        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


    }

    public void update() {

        imuYaw = MathUtil.angleWrap(imuYawOffset + orientation.firstAngle);

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        lifter.update();

        orientation = imu.getAngularOrientation();
        angularVelocity = imu.getAngularVelocity();
    }

    public void updateDrive(double LY, double LX, double RX, boolean FOD, boolean antiTip, boolean aprilTagAlignment, ButtonToggleAdvanced side) {
        drive.drive(LY, LX, RX,
                FOD, antiTip, aprilTagAlignment, side,
                getImuYaw(), // YAW
                orientation.secondAngle, // PITCH
                orientation.thirdAngle // ROLL
        );
    }

    public double getImuYaw() {
        return imuYaw;
    }

    public void setClawPosition(double clawPosition){
        Claw.setPosition(clawPosition);
    }

    public void setSlidePosition(int ticks) {
        lifter.pidController.resetTimer();
        lifter.setPosition(ticks);
    }

    public void changeSlidePosition(int ticks) {
        lifter.setPosition(ticks);
    }

//    public void setSlideLength(double inches) {
//        lifter.setLength(inches);
//    }

    public void resetIMU() {

    }
}
