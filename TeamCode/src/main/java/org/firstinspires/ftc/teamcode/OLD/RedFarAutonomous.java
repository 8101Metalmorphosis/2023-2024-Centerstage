package org.firstinspires.ftc.teamcode.OLD;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "Red Far Autonomous", group = "!")
public class RedFarAutonomous extends LinearOpMode {

    OpenCvCamera webcam;

    DcMotorEx leftLift, rightLift;

    DcMotorEx intake;

    Servo bucket;
    Servo dropper;

    public static int LIFTER_TOP = 720;
    public static int LIFTER_RESET = 20;

    public static float BUCKET_DROP = .7f;
    public static float BUCKET_RESET = .4f;

    public static int DROPPER_OPEN = 1;
    public static float DROPPER_CLOSE = .75f;

    @Override
    public void runOpMode() throws InterruptedException {

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        bucket = hardwareMap.get(Servo.class, "Bucket");
        dropper = hardwareMap.get(Servo.class, "Dropper");

        rightLift.setPower(-.1);
        sleep(1000);


        dropper.setDirection(Servo.Direction.REVERSE);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setTargetPosition(rightLift.getCurrentPosition());
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setTargetPosition(leftLift.getCurrentPosition());
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(.3);
        leftLift.setPower(.3);


        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        RedPipeline detector = new RedPipeline(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        String position = "MIDDLE";

        setLifterPosition(10);

        bucket.setPosition(BUCKET_RESET);
        dropper.setPosition(DROPPER_CLOSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory left = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(.5, -40))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .strafeLeft(8)
                .build();

        Trajectory middle2 = drive.trajectoryBuilder(middle.end())
                .splineToConstantHeading(new Vector2d(12, -31), Math.toRadians(90))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .strafeLeft(8)
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right.end())
                .splineToLinearHeading(new Pose2d(15, -31, Math.toRadians(0)), Math.toRadians(0))
                .build();


        int spike = 2;

        while(!isStarted()) {
            waitForStart();

            waitForStart();
            if (isStopRequested())
                return;

            switch (detector.getLocation()) {
                case RIGHT:
                    spike = 1;
                    telemetry.addData("Spike", "1");
                    break;
                case MIDDLE:
                    spike = 2;
                    telemetry.addData("Spike", "2");
                    break;
                case LEFT:
                    spike = 3;
                    telemetry.addData("Spike", "3");
                    break;
            }


            webcam.stopStreaming();
            telemetry.update();

            if(spike == 1) {
                drive.followTrajectory(right);
                drive.followTrajectory(right2);
            } else if(spike == 2) {
                drive.followTrajectory(middle);
                drive.followTrajectory(middle2);
            } else if(spike == 3) {
                drive.followTrajectory(left);
            }

            // outtake for a few ms
            intake.setPower(-.5);
            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineToSplineHeading(new Pose2d(16, -52, Math.toRadians(90)), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj2);
            sleep(1000);
            intake.setPower(0);
        }
    }


    public void setLifterPosition(int pos) {
        leftLift.setTargetPosition(pos);
        rightLift.setTargetPosition(pos);
    }
}
