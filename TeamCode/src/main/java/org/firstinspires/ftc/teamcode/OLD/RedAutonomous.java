package org.firstinspires.ftc.teamcode.OLD;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RedPipeline;
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
@Autonomous(name = "Red Autonomous", group = "!")
public class RedAutonomous extends LinearOpMode {

    OpenCvCamera webcam;

    DcMotorEx leftLift, rightLift;

    DcMotorEx intake;

    Servo bucket;
    Servo dropper;

    public static int LIFTER_TOP = 720;
    public static int LIFTER_RESET = 20;

    public static float BUCKET_DROP = .75f;
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
        Pose2d startPose = new Pose2d(16, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(9.25, -31, Math.toRadians(180)), Math.toRadians(190))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(12, -31), Math.toRadians(90))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(25, -42, Math.toRadians(90)), Math.toRadians(0))
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
            } else if(spike == 2) {
                drive.followTrajectory(middle);
            } else if(spike == 3) {
                drive.followTrajectory(left);
            }

            // outtake for a few ms
            intake.setPower(-.5);
            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .addSpatialMarker(new Vector2d(15, -34), () -> {
                        setLifterPosition(400);
                    })
                    .addSpatialMarker(new Vector2d(18, -35), () -> {
                        bucket.setPosition(BUCKET_DROP);
                    })
                    .addSpatialMarker(new Vector2d(22, -35), () -> {
                        setLifterPosition(LIFTER_TOP);
                    })
                    .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .addDisplacementMarker(5, () -> {
                        setLifterPosition(20);
                    })
                    .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                    .build();
            sleep(1000);
            drive.followTrajectory(traj2);
            sleep(1000);
            intake.setPower(0);
            dropper.setPosition(DROPPER_OPEN);
            sleep(1000);
            bucket.setPosition(BUCKET_RESET);
            sleep(250);
            dropper.setPosition(DROPPER_CLOSE);
            setLifterPosition(300);
            sleep(200);
            // move arm down
            drive.followTrajectory(traj3);
            sleep(5000);
        }
    }


    public void setLifterPosition(int pos) {
        leftLift.setTargetPosition(pos);
        rightLift.setTargetPosition(pos);
    }
}
