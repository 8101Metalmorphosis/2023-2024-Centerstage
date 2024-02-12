package org.firstinspires.ftc.teamcode.Autonomous.Red;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedBackdropAutonomous", group = "! a")
public class RedBackdropAutonomous extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(9, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0, () -> {
                    robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                })

                .addDisplacementMarker(8, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
                })
                .lineToSplineHeading(new Pose2d(36, -26, Math.toRadians(179.9)))
                .build();

        Trajectory left =  drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(18, -32))
                .build();

        Trajectory middle =  drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(32, -26))
                .build();

        Trajectory right =  drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(new Vector2d(41, -30))
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
                        .lineToConstantHeading(new Vector2d(45.5, -35.25))
                        .build();
            } else if (spike == 2) {
                drive.followTrajectory(middle);
                traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(2, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                        })
                        .lineToConstantHeading(new Vector2d(45.5, -38))
                        .build();
            } else if (spike == 3) {
                drive.followTrajectory(right);
                traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(2, () -> {
                            robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                        })
                        .lineToConstantHeading(new Vector2d(45.5, -42))
                        .build();
            }

            robot.setExtendArm((float) (Constants.ExtendConstants.intakeExtendArm + .3));
            sleep(100);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
            sleep(100);
            robot.setExtendArm((float) (Constants.ExtendConstants.intakeExtendArm + .3));
            sleep(100);
            robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
            robot.setIntake(Constants.IntakeConstants.autoOuttakeSpeed);
            sleep(1000);
            drive.followTrajectory(traj2);
            robot.setIntake(0);
            robot.lifter.claw.setPosition(Constants.ClawConstants.clawOpen);
            sleep(250);
            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2nd);
            sleep(250);
            robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);
            sleep(250);

            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(46, -66))
                    .build();
            drive.followTrajectory(traj3);
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(12)
                    .build();
            drive.followTrajectory(traj4);
        }
    }
}
