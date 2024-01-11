//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Constants;
//
//public class Extend {
////    public DcMotorEx extendLift;
//    public Servo extendArm;
//    public CRServo extendIntake;
//
//
//    public int extendPos;
//    public int targetPos;
//
//    public Extend(HardwareMap hardwareMap) {
////        extendLift = hardwareMap.get(DcMotorEx.class, "extendLift");
//
//        extendArm = hardwareMap.get(Servo.class, "extendArm");
//        extendIntake = hardwareMap.get(CRServo.class, "extendIntake");
//
//        resetExtend();
//    }
//
//    public void zeroExtend() {
////        extendLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        extendLift.setPower(-.1);
////        resetExtend();
//    }
//
//    public void resetExtend() {
////        extendLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
////        extendLift.setDirection(DcMotorSimple.Direction.REVERSE);
////
////        extendLift.setTargetPosition(extendLift.getCurrentPosition());
////        extendLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////
////        extendLift.setPower(Constants.ExtendConstants.extendSpeed);
//    }
//
//    public void setExtendPosition(int ticks) {
////        targetPos = ticks;
//
////        extendLift.setTargetPosition(ticks);
//    }
//
//    public void setArmPosition(float position) {
//        extendArm.setPosition(position);
//    }
//
//    public void setIntakeSpeed(float speed) {
//        extendIntake.setPower(speed);
//    }
//
//    public void update() {
//        extendPos = extendLift.getCurrentPosition();
//    }
//
//    public double getPower() {
//        return extendLift.getPower();
//    }
//}
