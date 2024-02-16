package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
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


    enum State {
        LEFT,
        MIDDLE,
        RIGHT,
        OUTTAKE,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        RIGHT_2,
        AUTO_TARGET_1,
        TRAJECTORY_4,
        TRAJECTORY_5,
        TRAJECTORY_6,
        AUTO_TARGET_2,
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

        robot = new RobotBase(hardwareMap, false);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory left = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(4, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-46, -16, Math.toRadians(-90)))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(4, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-46, -13, Math.toRadians(-45)))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(4, () -> {
                    robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
                })
                .lineToLinearHeading(new Pose2d(-41, -34, Math.toRadians(0)))
                .build();


        Trajectory traj1 = drive.trajectoryBuilder(middle.end())
                .addDisplacementMarker(1, () -> {
                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                    robot.setExtendArm(Constants.ExtendConstants.stackIntake5 + .04f);
                })
                .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
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
                .splineToLinearHeading(new Pose2d(35, -18, Math.toRadians(158)), Math.toRadians(-15))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(36, -32, Math.toRadians(155)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(36, -14, Math.toRadians(180 + 1e-6)), Math.toRadians(180 + 1e-6))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-52, -14, Math.toRadians(180 + 1e-6)), Math.toRadians(180 + 1e-6))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
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
                .splineToLinearHeading(new Pose2d(35, -18, Math.toRadians(158)), Math.toRadians(-15))
                .build();

        Trajectory park = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                .build();



        int spike = 2;
        int mainTag = 5;
        float tagY = 42.5f;


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
            mainTag = 4;
            tagY = -35.5f;
            currentState = State.LEFT;
            drive.followTrajectoryAsync(left);
        } else if (spike == 2) {
            mainTag = 5;
            tagY = -42f;
            currentState = State.MIDDLE;
            drive.followTrajectoryAsync(middle);
        } else if (spike == 3) {
            mainTag = 6;
            tagY = -48f;
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
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
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
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
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
                        .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
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

                        if(spike == 3) {
                            drive.followTrajectoryAsync(right2);
                            currentState = State.RIGHT_2;
                        }

                    }

                    break;



                case RIGHT_2:

                    if(!drive.isBusy()) {
                        stateTimer.reset();
                        currentState = State.AUTO_TARGET_1;

                    }
                    break;


                case AUTO_TARGET_1:

                    if(spike == 1 || spike == 2) {
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


                            drive.setPoseEstimate(robot.drive.calculatePose(robot.drive.getScannedTag(mainTag)), new Vector2d(62.5, tagY));


                            traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .addDisplacementMarker(0, () -> {
                                        robot.extend.closeIntakeDoor();
                                    })
                                    .addDisplacementMarker(6, () -> {
                                        robot.setLifterArm(Constants.LifterConstants.liftArmIntake + .15f);
                                        robot.extend.setArmPosition(Constants.ExtendConstants.intakeExtendArm);
                                    })
                                    .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(180 - 1e-6)))
                                    .build();

                            autoTargeting = false;
                            stateTimer.reset();
                            drive.followTrajectoryAsync(traj4);
                            currentState = State.TRAJECTORY_4;
                        }
                    }
                    else if (stateTimer.milliseconds() >= 12000) {
                        robot.setLifterArm(Constants.LifterConstants.liftArmIdle);

                        drive.setPoseEstimate(robot.drive.calculatePose(robot.drive.getScannedTag(mainTag)), new Vector2d(62.5, tagY));

                        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                                .build();
                      autoTargeting = false;
                      stateTimer.reset();
                      drive.followTrajectoryAsync(park);
                      currentState = State.PARK;
                    }

                    break;


                case TRAJECTORY_4:
                    if(!drive.isBusy()) {
                        traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-54, -14, Math.toRadians(-175)))
                                .addDisplacementMarker(10, () -> {
                                    robot.extend.setArmPosition(Constants.ExtendConstants.stackIntake5 + .075f);
                                    robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                                })
                                .build();
                        stateTimer.reset();
                        drive.followTrajectoryAsync(traj5);
                        currentState = State.TRAJECTORY_5;
                    }
                    break;

                case TRAJECTORY_5:
                    if(stateTimer.milliseconds() >= 100) {
                        robot.extend.setArmPosition(Constants.ExtendConstants.stackIntake4);
                    }
                    if(stateTimer.milliseconds() >= 400) {
                            traj6 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
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
                                    .splineToLinearHeading(new Pose2d(35, -18, Math.toRadians(158)), Math.toRadians(-15))
                                    .build();
                            stateTimer.reset();
                            drive.followTrajectory(traj6);
                            currentState = State.TRAJECTORY_6;
                        }
                    break;

                case TRAJECTORY_6:
                    if(!drive.isBusy()) {
                        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                                .build();
                        stateTimer.reset();
                        drive.followTrajectory(park);
                        currentState = State.PARK;
                    }

                case AUTO_TARGET_2:
                    autoTargeting = true;

                    if(robot.drive.scanForTag(4)) {
                        robot.drive.alignToTargetTag(4);
                    } else {
                        robot.drive.setZero();
                    }

                    if(robot.drive.scanForTag(4)) {
                        if(MathUtil.isInRange(robot.drive.getScannedTag(4).ftcPose.range, 10)) {
                            robot.lifter.claw.setPosition(Constants.ClawConstants.clawOpen);

                            autoTargeting = false;

                            drive.setPoseEstimate(robot.drive.calculatePose(robot.drive.getScannedTag(mainTag)), new Vector2d(62.5, tagY));

                            park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                                    .build();
                            autoTargeting = false;
                            stateTimer.reset();
                            drive.followTrajectoryAsync(park);
                            currentState = State.PARK;
                        }
                    }
                    else if (stateTimer.milliseconds() >= 12000) {
                        drive.setPoseEstimate(robot.drive.calculatePose(robot.drive.getScannedTag(mainTag)), new Vector2d(62.5, tagY));

                        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180 + 1e-6)), Math.toRadians(0))
                                .build();
                        autoTargeting = false;
                        stateTimer.reset();
                        drive.followTrajectoryAsync(park);
                        currentState = State.PARK;
                    }

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



