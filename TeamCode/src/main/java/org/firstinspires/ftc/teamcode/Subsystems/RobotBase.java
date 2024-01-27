package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    public MecanumDrive drive;
    public Lifter lifter;
    public Extend extend;

    public double imuYawOffset = 0;

    double imuYaw = 0;


    List<LynxModule> allHubs;


    public RobotBase(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drive = new MecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap);
        extend = new Extend(hardwareMap);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());


        orientation = imu.getAngularOrientation();
        angularVelocity = imu.getAngularVelocity();


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
        // Needs bulk reading

        lifter.update();
        extend.update();

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


    // Lifter
    public void setLifter(int targetPosition) {
        lifter.setLifterPosition(targetPosition);
    }

    public void setLifterArm(float targetPosition) {
        lifter.setArmPosition(targetPosition);
    }

    public void setLifterWrist(float targetPitchPosition, float targetRollPosition) {
        lifter.setWristPitchPosition(targetPitchPosition);
        lifter.setWristRollPosition(targetRollPosition);
    }

    public void setLifterWristPitch(float targetPitchPosition) {
        lifter.setWristPitchPosition(targetPitchPosition);
    }

    public void setLifterWristRoll(float targetRollPosition) {
        lifter.setWristRollPosition(targetRollPosition);
    }

    public void setLifterSystem(int targetLiftPosition, float targetArmPosition, float targetWristPosition) {
        lifter.setLifterPosition(targetLiftPosition);
        lifter.setArmPosition(targetArmPosition);
        lifter.setWristPitchPosition(targetWristPosition);
    }

    // Extend
    public void setExtend(int targetPosition) {
        extend.setExtendPosition(targetPosition);
    }

    public void setExtendArm(float targetPosition) {
        extend.setArmPosition(targetPosition);
    }

//    public void updateExtend(int targetPosition) {
//        extend.setExtendPosition(targetPosition);
//    }
//
//    public void updateExtendArm(float targetPosition) {
//        extend.setArmPosition(targetPosition);
//    }
//
//    public void updateExtendSystem(int targetLiftPosition, float targetArmPosition) {
//        extend.setExtendPosition(targetLiftPosition);
//        extend.setArmPosition(targetArmPosition);
//    }

    public void setIntake(float speed) {
        extend.setIntakeSpeed(speed);
    }

    public double getImuYaw() {
        return imuYaw;
    }



    public void resetIMU() {

    }
}
