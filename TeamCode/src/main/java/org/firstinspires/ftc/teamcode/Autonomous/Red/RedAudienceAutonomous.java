package org.firstinspires.ftc.teamcode.Autonomous.Red;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Extend;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Audience Autonomous", group = "! a")
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
        Pose2d startPose = new Pose2d(-40, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0, () -> {
                    robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                })
                .lineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(0)))
                .build();

        Trajectory leftSpike = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-46, -18.5, Math.toRadians(-90)))
                .build();

        Trajectory middleSpike = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-56, -26))
                .build();

//        Trajectory rightSpike = drive.trajectoryBuilder(traj1.end())
//
//                .build();


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

            robot.extend.closeIntakeDoor();
            robot.lifter.clawClose();

            drive.followTrajectory(traj1);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);

            if(spike == 1) {
                drive.followTrajectory(leftSpike);
            } else if (spike == 2) {
                drive.followTrajectory(middleSpike);
            } else if (spike == 3) {

            }

            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(4, () -> {
                        robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                        robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(179.9)))
                    .build();

            robot.setIntake(Constants.IntakeConstants.autoOuttakeSpeed);
            sleep(800);
            robot.setIntake(0);

            drive.followTrajectory(traj2);
            robot.setIntake(Constants.IntakeConstants.intakeSpeed);

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .forward(2)
                    .build();

            drive.followTrajectory(traj3);
            sleep(100);
            robot.setIntake(0);
            robot.setExtendArm(Constants.ExtendConstants.resetExtendArm);


            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .back(20)
                    .build();

            if(spike == 1) {
                traj4 = drive.trajectoryBuilder(traj3.end(), true)
                        .splineToConstantHeading(new Vector2d(45.5, -28), Math.toRadians(-45))
                        .addDisplacementMarker(4, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                        })
                        .addDisplacementMarker(10, () -> {
                            robot.extend.openIntakeDoor();
                            robot.lifter.clawOpen();
                        })
                        .addDisplacementMarker(15, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);
                        })
                        .addDisplacementMarker(45, () -> {
                            robot.lifter.clawClose();
                        })
                        .addDisplacementMarker(55, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                        })
                        .addDisplacementMarker(60, () -> {
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollRight);
                        })
                        .build();
            } else if (spike == 2) {
                traj4 = drive.trajectoryBuilder(traj3.end(), true)
                        .splineToConstantHeading(new Vector2d(45.5, -33), Math.toRadians(-45))
                        .addDisplacementMarker(4, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                        })
                        .addDisplacementMarker(10, () -> {
                            robot.extend.openIntakeDoor();
                            robot.lifter.clawOpen();
                        })
                        .addDisplacementMarker(15, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);
                        })
                        .addDisplacementMarker(45, () -> {
                            robot.lifter.clawClose();
                        })
                        .addDisplacementMarker(55, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                        })
                        .addDisplacementMarker(60, () -> {
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                        })
                        .build();
            }
            sleep(100);
            drive.followTrajectory(traj4);
            robot.extend.closeIntakeDoor();
            sleep(250);
            robot.lifter.clawOpen();
            sleep(250);
            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2nd - .05f);
            sleep(250);
            robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
            sleep(10000);


        }
    }
}
