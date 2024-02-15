package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Extend;
import org.firstinspires.ftc.teamcode.Vision.BluePipeline;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Audience Autonomous", group = "! a")
public class BlueAudienceAutonomous extends LinearOpMode {


    RobotBase robot;

    OpenCvCamera webcam;


    enum State {
        LEFT,
        MIDDLE,
        RIGHT,
        OUTTAKE,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        AUTO_TARGET_1,
        PARK,
        IDLE
    }


    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BluePipeline detector = new BluePipeline(telemetry);
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

        robot = new RobotBase(hardwareMap, false);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-32, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);


        Trajectory left = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-47.5, 16, Math.toRadians(90)))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(45)))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-41, 34, Math.toRadians(0)))
                .build();


        Trajectory traj1 = drive.trajectoryBuilder(middle.end())
                .addDisplacementMarker(1, () -> {
                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5 + .075f);
                })
                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180 - 1e-6)))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(0, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5 + .04f);
                })
                .forward(2.25)
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .addDisplacementMarker(1, () -> {
                    robot.setIntake(0);
                })
                .addDisplacementMarker(1, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.resetExtendArm);
                })
                .addDisplacementMarker(10, () -> {
                    robot.lifter.clawOpen();
                })
                .addDisplacementMarker(19, () -> {
                    robot.extend.openIntakeDoor();
                })
                .addDisplacementMarker(19, () -> {
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
                .splineToLinearHeading(new Pose2d(36, 22, Math.toRadians(-160)), Math.toRadians(30))
                .build();


        Trajectory park = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                .build();



        int spike = 2;
        int mainTag = 5;


        boolean autoTargeting = false;


        ElapsedTime stateTimer = new ElapsedTime();


        waitForStart();

        if (isStopRequested()) return;

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



        if(spike == 1) {
            mainTag = 1;
            currentState = State.LEFT;
            drive.followTrajectoryAsync(left);
        } else if (spike == 2) {
            mainTag = 2;
            currentState = State.MIDDLE;
            drive.followTrajectoryAsync(middle);
        } else if (spike == 3) {
            mainTag = 3;
            currentState = State.RIGHT;
            drive.followTrajectoryAsync(right);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        robot.drive.initAprilTag(hardwareMap);

        robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
        robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
        robot.extend.closeIntakeDoor();

        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {

                case LEFT:

                    if(!drive.isBusy()) {
                        traj1 = drive.trajectoryBuilder(left.end())
                                .addDisplacementMarker(1, () -> {
                                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                                })
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180 - 1e-6)))
                                .build();

                        stateTimer.reset();
                        currentState = State.OUTTAKE;
                    }

                    break;


                case MIDDLE:

                    if(!drive.isBusy()) {
                        traj1 = drive.trajectoryBuilder(middle.end())
                                .addDisplacementMarker(1, () -> {
                                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                                })
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180 - 1e-6)))
                                .build();

                        stateTimer.reset();
                        currentState = State.OUTTAKE;
                    }

                    break;


                case RIGHT:

                    if(!drive.isBusy()) {
                        traj1 = drive.trajectoryBuilder(right.end())
                                .addDisplacementMarker(1, () -> {
                                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5);
                                })
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180 - 1e-6)))
                                .build();

                        stateTimer.reset();
                        currentState = State.OUTTAKE;
                    }

                    break;

                case OUTTAKE:

                    robot.setIntake(Constants.IntakeConstants.autoOuttakeSpeed);

                    if(stateTimer.milliseconds() >= 750) {
                        robot.setIntake(0);
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
                        currentState = State.TRAJECTORY_3;
                        robot.drive.setManualExposure(25, 8);
                    }

                    break;


                case TRAJECTORY_3:

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        currentState = State.AUTO_TARGET_1;

                    }

                    break;


                case AUTO_TARGET_1:

                    if(spike == 1) {
                        robot.setLifterWristRoll(Constants.ClawConstants.wristRollRight);
                    } else {
                        robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                    }

                    autoTargeting = true;

                    if(robot.drive.scanForTag(mainTag)) {
                        robot.drive.alignToTargetTag(mainTag);
                    } else {
                        robot.drive.setZero();
                    }

                    if(robot.drive.scanForTag(mainTag)) {
                        if(MathUtil.isInRange(robot.drive.getScannedTag(mainTag).ftcPose.range, 10)) {
                            robot.lifter.claw.setPosition(Constants.ClawConstants.clawOpen);

                            autoTargeting = false;
                            stateTimer.reset();

                            if(mainTag == 4) {
                                drive.setPoseEstimate(new Pose2d(44, 28, Math.toRadians(-175)));
                            } else if (mainTag == 5) {
                                drive.setPoseEstimate(new Pose2d(44, 32, Math.toRadians(-175)));
                            } else {
                                drive.setPoseEstimate(new Pose2d(44, 42, Math.toRadians(-175)));
                            }

                            park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .splineToLinearHeading(new Pose2d(56, 12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                                    .build();
                            drive.followTrajectoryAsync(park);
                            currentState = State.PARK;
                        }
                    }
//                    else if (stateTimer.milliseconds() >= 12000) {]\

//                      autoTargeting = false;
//                      stateTimer.reset();
////                      drive.followTrajectoryAsync(park);
////                      currentState = State.PARK;
//                    }

                    break;


                case PARK:

                    if(stateTimer.milliseconds() >= 250) {
                        robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);

                        if(stateTimer.milliseconds() >= 400) {
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
                            robot.setLifterWristRoll(Constants.ClawConstants.wristRollVertical);
                        }
                    }

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        currentState = State.IDLE;
                    }

                    break;

                case IDLE:
                    break;
            }


            if(!autoTargeting) {
                drive.update();
            }
        }
    }
}



