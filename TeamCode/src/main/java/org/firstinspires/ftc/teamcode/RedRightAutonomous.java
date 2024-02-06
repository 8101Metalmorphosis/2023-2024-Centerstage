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

@Autonomous(name = "RedRightAutonomous", group = "!")
public class RedRightAutonomous extends LinearOpMode {

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
                .addDisplacementMarker(0, () -> {
                    robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                })

                .addSpatialMarker(new Vector2d(20, -30), () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
                })
                .lineToSplineHeading(new Pose2d(28, -31, Math.toRadians(180 + (1e-6))))
                .build();

        Trajectory left =  drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(11.25, -34))
                .build();

        Trajectory middle =  drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(27, -29))
                .build();

        Trajectory right =  drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(new Vector2d(33, -40))
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

            robot.extend.closeIntakeDoor();
            robot.lifter.clawClose();

            drive.followTrajectory(traj1);


            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(39.5, -39.5))
                    .build();

            if(spike == 1) {
                drive.followTrajectory(left);
                traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(2, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollRight);
                        })
                        .lineToConstantHeading(new Vector2d(39.75, -36.5))
                        .build();
            } else if (spike == 2) {
                drive.followTrajectory(middle);
                traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(2, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                        })
                        .lineToConstantHeading(new Vector2d(39.75, -39.75))
                        .build();

            } else if (spike == 3) {
                drive.followTrajectory(right);
                traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(2, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                        })
                        .lineToConstantHeading(new Vector2d(39.75, -45))
                        .build();
            }

            robot.setExtendArm((float) (Constants.ExtendConstants.intakeExtendArm + .2));
            sleep(100);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
            robot.setIntake(-.25f);

            sleep(2500);
            drive.followTrajectory(traj2);
            robot.setIntake(0);
            robot.lifter.claw.setPosition(Constants.ClawConstants.clawFullOpen);
            sleep(500);
            robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);
            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
            sleep(2500);

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(40, -68))
                    .build();
            drive.followTrajectory(traj3);
//            drive.followTrajectory();
        }
    }
}
