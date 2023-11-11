package org.firstinspires.ftc.teamcode.NikoRunner;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NikoRunner.library.Vector2d;

public class TwoWheelLocalizer {

    public BNO055IMU imu;
    public DcMotorEx par, perp;

    public Vector2d countPose;
    public Vector2d pose;

    public double inPerTick;


    public int prevParCount;
    public int prevPerpCount;

    public int changeInParCount;
    public int changeInPerpCount;

    public TwoWheelLocalizer(HardwareMap hardwareMap, DcMotorEx par, DcMotorEx perp, BNO055IMU imu, double inPerTick) {
        this.par = par;
        this.perp = perp;

        prevParCount = par.getCurrentPosition();
        prevPerpCount = perp.getCurrentPosition();

        this.imu = imu;

        this.inPerTick = inPerTick;
    }

    public Vector2d getPose() {
        return pose;
    }

    public void update() {
        int parCount = par.getCurrentPosition();
        int perpCount = perp.getCurrentPosition();

        changeInParCount = parCount - prevParCount;
        changeInPerpCount = perpCount - prevPerpCount;

        countPose = new Vector2d(
            (parCount * Math.cos(imu.getAngularOrientation().thirdAngle)) - (perpCount * Math.sin(imu.getAngularOrientation().thirdAngle)),
            (parCount * Math.sin(imu.getAngularOrientation().thirdAngle)) + (perpCount * Math.cos(imu.getAngularOrientation().thirdAngle))
        );

        pose = new Vector2d((countPose.getX() * inPerTick), (countPose.getY() * inPerTick));

        prevParCount = parCount;
        prevPerpCount = perpCount;
    }
}