package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.FTCutil.PID.PIDController;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class MecanumDrive {

    PIDController headingController = new PIDController(1, 0, 0, 0);


    public List<AprilTagDetection> currentDetections;


    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx BackLeft;
    DcMotorEx BackRight;

    public double frontLeftPos;
    public double frontRightPos;
    public double backLeftPos;
    public double backRightPos;


    public MecanumDrive(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");


        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        frontLeftPos = FrontLeft.getCurrentPosition();
        frontRightPos = FrontRight.getCurrentPosition();
        backLeftPos = BackLeft.getCurrentPosition();
        backRightPos = BackRight.getCurrentPosition();
    }

    public void drive(double LY, double LX, double RX, boolean FOD, boolean antiTip, boolean alignAprilTag,
                      ButtonToggleAdvanced side, double yaw, double pitch, double roll) {

        if(alignAprilTag) {

            // PUT APRIL TAG DETECTION IN HERE
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if(side.getState() == ButtonToggleAdvanced.OFF_STATE) { // RED SIDE
                        if(detection.id == AprilTags.ID_REDALLIANCE_WALLSMALL) {
                            alignToTag(detection);
                        } else if(detection.id == AprilTags.ID_REDALLIANCE_CENTER) {
                            alignToTag(detection);
                        }
                    } else if (side.getState() == ButtonToggleAdvanced.ON_STATE) { // BLUE SIDE
                        if(detection.id == AprilTags.ID_BLUEALLIANCE_WALLSMALL) {
                            alignToTag(detection);
                        } else if(detection.id == AprilTags.ID_BLUEALLIANCE_CENTER) {
                            alignToTag(detection);
                        }
                    } else {
                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                        BackLeft.setPower(0);
                        BackRight.setPower(0);
                    }
                } else {
                    FrontLeft.setPower(0);
                    FrontRight.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                }
            }
            return;
        }

        if (FOD) {
            Vector2d rotatedValues = MathUtil.rotateByAngle(new Vector2d(LX, LY), -yaw);

            LX = rotatedValues.getX();
            LY = rotatedValues.getY();
        }

//        if (!antiTip) {
//            yaw = 0;
//            pitch = 0;
//            roll = 0;
//        }

//        if (MathUtil.isInRange(yaw, Chassis.rotationConstraintsRAD[0]) || MathUtil.isInRange(roll, Chassis.rotationConstraintsRAD[1])) {
//            yaw = yaw * 3;
//            roll = roll * 3;
//        } else {
//            yaw = 0;
//            roll = 0;
//        }
//
//        yaw = yaw * 3;
//        roll = roll * 3;

        FrontLeft.setPower((LY + LX + RX));
        FrontRight.setPower((LY - LX - RX));
        BackLeft.setPower((LY - LX + RX));
        BackRight.setPower((LY + LX - RX));
    }

    public void alignToTag(AprilTagDetection detection) {
        double targetDistance = 8;

        double DISTANCE_GAIN = 0.036;
        double STRAFE_GAIN = 0.02;
        double TURN_GAIN = 0.025;

        double MAX_DRIVE_SPEED = .4;
        double MAX_STRAFE_SPEED = .2;
        double MAX_TURN_SPEED = .1;

        // DRIVE
        double rangeError = detection.ftcPose.range - targetDistance;
        double rangeValue = Range.clip(rangeError * DISTANCE_GAIN, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        // STRAFE
        double yawError = detection.ftcPose.yaw;
        double yawValue = Range.clip(-yawError * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

        // TURN
        double headingError = detection.ftcPose.bearing;
        double headingValue = Range.clip(headingError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        FrontLeft.setPower(-rangeValue + yawValue - headingValue);
        FrontRight.setPower(-rangeValue - yawValue + headingValue);
        BackLeft.setPower(-rangeValue - yawValue - headingValue);
        BackRight.setPower(-rangeValue + yawValue + headingValue);
    }
}
