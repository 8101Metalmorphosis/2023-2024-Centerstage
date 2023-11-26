package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;

import java.util.List;

public class RobotBase {

    HardwareMap hardwareMap;

    // IMU
    BNO055IMU imu;

    public Orientation orientation;
    public AngularVelocity angularVelocity;

    // Motors
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;

    public DcMotorEx intake;

    public Servo bucket;
    public Servo dropper;

    public MecanumDrive drive;
    public Lifter lifter;

    int FrontLeftPosition = 0;
    int FrontRightPosition = 0;
    int BackLeftPosition = 0;
    int BackRightPosition = 0;

    public double imuYawOffset = 0;

    double imuYaw = 0;


    List<LynxModule> allHubs;



    public RobotBase(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drive = new MecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());


        orientation = imu.getAngularOrientation();
        angularVelocity = imu.getAngularVelocity();



        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        bucket = hardwareMap.get(Servo.class, "Bucket");
        dropper = hardwareMap.get(Servo.class, "Dropper");



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

    public void changeArmPosition(int ticks) {
        lifter.pidController.resetTimer();
        lifter.setPosition(ticks);
    }

    public void setArmPosition(int ticks) {
        lifter.setPosition(ticks);

    }

    public void intakeOn(){
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeOut() {
        intake.setPower(-1);
    }

    public void resetIMU() {

    }
}
