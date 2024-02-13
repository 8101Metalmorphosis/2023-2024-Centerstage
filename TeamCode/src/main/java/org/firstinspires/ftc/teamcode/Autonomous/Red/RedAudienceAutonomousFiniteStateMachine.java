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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Audience Autonomous Finite State Machine", group = "! a")
public class RedAudienceAutonomousFiniteStateMachine extends LinearOpMode {

    RobotBase robot;

    OpenCvCamera webcam;


    enum State {
        LEFT,
        MIDDLE,
        RIGHT,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        AUTO_TARGET_1,
        IDLE
    }

    State currentState = State.IDLE;




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

        Trajectory leftSpike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46, -18.5, Math.toRadians(-90)))
                .build();

        Trajectory middleSpike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56, -26, Math.toRadians(-90)))
                .build();

        Trajectory rightSpike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(0)))
                .build();



        Trajectory traj1 = drive.trajectoryBuilder(middleSpike.end())
                .addDisplacementMarker(4, () -> {
                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                })
                .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(2)
                .build();

         Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
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
                .splineToLinearHeading(new Pose2d(28, -12, Math.toRadians(-45)), Math.toRadians(0))
                .build();



        int spike = 2;
        
        
        ElapsedTimer stateTimer = new ElapsedTime();

        waitForStart();


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


        if(spike == 1) {
            currentState = State.LEFT;
            drive.followTrajectoryAsync(left);
        } else if (spike == 2) {
            currentState = State.MIDDLE;
            drive.followTrajectoryAsync(middle);
        } else if (spike == 3) {
            currentState = State.RIGHT;
            drive.followTrajectoryAsync(right);
        }

        
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {

                case LEFT:

                    if(!drive.isBusy()) {
                        Trajectory traj1 = drive.trajectoryBuilder(leftSpike.end())
                        .addDisplacementMarker(4, () -> {
                            robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                            robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                        })
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
                        .build();


                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj1);
                        currentState = State.TRAJECTORY_1;
                    }

                    break;

                case MIDDLE:

                    if(!drive.isBusy()) {
                        Trajectory traj1 = drive.trajectoryBuilder(middleSpike.end())
                        .addDisplacementMarker(4, () -> {
                            robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                            robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                        })
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
                        .build();


                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj1);
                        currentState = State.TRAJECTORY_1;
                    }
                    
                    break;

                case RIGHT:
                
                    if(!drive.isBusy()) {
                        Trajectory traj1 = drive.trajectoryBuilder(rightSpike.end())
                        .addDisplacementMarker(4, () -> {
                            robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                            robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                        })
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
                        .build();


                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj1);
                        currentState = State.TRAJECTORY_1;
                    }
                
                    break;


                case TRAJECTORY_1:
                    
                    if(!drive.isBusy()) {


                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj2);
                        currentState = State.TRAJECTORY_2;
                    }
                    break;

                case TRAJECTORY_2:

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj3);
                        currentState = State.TRAJECTORY_2;
                    }
                    break;

                case TRAJECTORY_3:

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        currentState = State.AUTO_TARGET_1;
                    }
                    break;

                case AUTO_TARGET_1:

                    if(robot.drive.getScannedTag(AprilTags.spikeToAprilTagRED.get(spike)).id != null) {

                    }

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj4);
                        currentState = State.TRAJECTORY_4;
                    }
                    break;




                
                case IDLE:
                    break;
            }

        }

        
    }
}
