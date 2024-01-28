package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OLD.RedPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedLeft extends LinearOpMode {

    RobotBase robot;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotBase(hardwareMap, false);


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


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-42,  -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-37, -16, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(middle.end())
                .splineToSplineHeading(new Pose2d(-59, -12, Math.toRadians(180) - 1e-6f), Math.toRadians(180))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(28, -12))
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

            } else if(spike == 2) {
                drive.followTrajectory(middle);
            } else if(spike == 3) {

            }

            // outtake for a few ms
            robot.setIntake(-Constants.IntakeConstants.intakeSpeed);
            sleep(350);
        }
    }
}
