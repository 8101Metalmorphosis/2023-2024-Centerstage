package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Audience Autonomous", group = "!")
public class RedAudienceAutonomous extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(8, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(0)))
                .build();

        Trajectory leftSpike = drive.trajectoryBuilder(traj1.end())

                .build();

        Trajectory middleSpike = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-45, -24))
                .build();

        Trajectory rightSpike = drive.trajectoryBuilder(traj1.end())

                .build();


        int spike = 2;

        while(!isStarted()) {
            waitForStart();

            waitForStart();
            if (isStopRequested())
                return;

            switch (detector.getLocation()) {
                case LEFT:
                    spike = 1;
                    telemetry.addData("Spike", "1");
                    break;
                case MIDDLE:
                    spike = 2;
                    telemetry.addData("Spike", "2");
                    break;
                case RIGHT:
                    spike = 3;
                    telemetry.addData("Spike", "3");
                    break;
            }

            drive.followTrajectory(traj1);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
            drive.followTrajectory(middleSpike);
            robot.setIntake(Constants.IntakeConstants.autoOuttakeSpeed);
            sleep(250);

            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(2, () -> {
                        robot.setIntake(-Constants.IntakeConstants.intakeSpeed);
                    })
                    .addDisplacementMarker(4, () -> {
                        robot.setExtendArm(Constants.ExtendConstants.stackIntakeExtendArm);
                        robot.setIntake(0);
                    })
                    .lineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180 + 1e-6)))
                    .build();

            drive.followTrajectory(traj2);

            robot.setIntake(Constants.IntakeConstants.intakeSpeed);
            robot.setExtendArm(Constants.ExtendConstants.stackIntakeExtendArm - (Constants.ExtendConstants.stackIntakeStep));
            sleep(250);

            robot.setIntake(0);
            robot.set


        }
    }
}
