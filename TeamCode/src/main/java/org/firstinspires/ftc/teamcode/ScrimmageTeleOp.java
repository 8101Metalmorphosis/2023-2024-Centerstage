package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Scrimmage TeleOp", group = "!")
public class ScrimmageTeleOp extends LinearOpMode {

    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx BackLeft;
    private DcMotorEx BackRight;

    private DcMotorEx Slides1;
    private DcMotorEx Slides2;

    private IMU imu;

    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    boolean FOD = false;



    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        Slides1 = hardwareMap.get(DcMotorEx.class, "Slides1");
        Slides2 = hardwareMap.get(DcMotorEx.class, "Slides2");

        imu = hardwareMap.get(IMU.class, "imu");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));


        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);



        // TESTING


        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {

                // Orientation goes from -90 to 90 instead of -180 to 180 for some reason NEEDS FIX
                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


                yaw = orientation.getYaw(AngleUnit.RADIANS);
                pitch = orientation.getPitch(AngleUnit.RADIANS);
                roll = orientation.getRoll(AngleUnit.RADIANS);

                double LY = -gamepad1.left_stick_y;
                double LX = gamepad1.left_stick_x;
                double RX = gamepad1.right_stick_x;

                telemetry.addData("LY", LY);
                telemetry.addData("LX", LX);
                telemetry.addData("RX", RX);

                Mecanum(LY, LX, RX);


                if(gamepad1.a) {
                    FOD = !FOD;
                    sleep(25);
                }

                if(gamepad1.start) {
                    imu.resetYaw();
                }


            }
        }
    }

    public void Mecanum(double LY, double LX, double RX) {
        telemetry.addData("FOD", FOD);

        // WILL BE ROBOT IMU ROTATION IN RADIANS
        double robotRotation = yaw;

        if (FOD) {
            double yValue = LY * Math.cos(robotRotation) - LX * Math.sin(robotRotation);
            double xValue = LY * Math.sin(robotRotation) + LX * Math.cos(robotRotation);

            LY = yValue;
            LX = xValue;
        }

        FrontLeft.setPower(LY + LX + RX);
        FrontRight.setPower(LY - LX - RX);
        BackLeft.setPower(LY - LX + RX);
        BackRight.setPower(LY + LX - RX);

        telemetry.addData("FL", FrontLeft.getPower());
        telemetry.addData("FR", FrontRight.getPower());
        telemetry.addData("BL", BackLeft.getPower());
        telemetry.addData("BR", BackRight.getPower());
        telemetry.addData("LY", LY);
        telemetry.addData("LX", LX);
        telemetry.addData("Yaw: ", Math.toDegrees(yaw));
        telemetry.update();
    }
}
