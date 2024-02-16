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

@Autonomous(name = "Red Backdrop Autonomous Finite State Machine", group = "! a")
public class RedBackdropAutonomousFiniteStateMachine extends LinearOpMode {

   RobotBase robot;

   OpenCvCamera webcam;


   enum State {
       LEFT,
       MIDDLE,
       RIGHT,
       TRAJECTORY_1,
       AUTO_TARGET_1,
       PARK,
       IDLE
   }


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



        Trajectory left =  drive.trajectoryBuilder(traj1.end())
            .addDisplacementMarker(4, () -> {
                robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
            })
            .lineToConstantHeading(new Vector2d(18, -32))
            .build();

        Trajectory middle =  drive.trajectoryBuilder(traj1.end())
            .addDisplacementMarker(4, () -> {
                robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
            })
            .lineToConstantHeading(new Vector2d(32, -26))
            .build();

        Trajectory right =  drive.trajectoryBuilder(traj1.end(), true)
            .addDisplacementMarker(4, () -> {
                robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm + .04f);
            })
            .lineToConstantHeading(new Vector2d(41, -30))
            .build();


       Trajectory traj1 = drive.trajectoryBuilder(traj1.end(), true)
               .lineToConstantHeading(new Vector2d(42, -36))
               .build();


       int spike = 2;


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
       }


       if (isStopRequested()) return;

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
                   robot.extend.closeIntakeDoor();
                   robot.lifter.clawClose();
                   robot.setLifterArm(Constants.LifterConstants.liftArmIdle);

                   if(!drive.isBusy()) {
                       Trajectory traj1 = drive.trajectoryBuilder(leftSpike.end())
                       .addDisplacementMarker(4, () -> {
                           robot.setExtendArm(Constants.ExtendConstants.intakeExtendArm);
                       })
                       .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180 - 1e-6)))
                       .build();


                   stateTimer.reset();
                   drive.followTrajectoryAsync(traj1);
                   currentState = State.TRAJECTORY_1;
                   }

                   break;

               case MIDDLE:
                   robot.extend.closeIntakeDoor();
                   robot.lifter.clawClose();
                   robot.setLifterArm(Constants.LifterConstants.liftArmIdle);

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
                   robot.extend.closeIntakeDoor();
                   robot.lifter.clawClose();
                   robot.setLifterArm(Constants.LifterConstants.liftArmIdle);

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

               case OUTTAKE:

                   robot.setIntake(Constants.IntakeConstants.autoOuttake)
                   robot.setLifterArm(Constants.LifterConstants.liftArmTop2)
                   robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2)

                   if(spike == 1) {
                       robot.setLifterWristRoll(Constants.ClawConstants.wristRollRight);
                   } else {
                       robot.setLifterWristRoll(Constants.ClawConstants.wristRollLeft);
                   }

                   if(stateTimer.milliseconds() >= 1000) {
                       robot.setIntake(0)
                       stateTimer.reset();
                       drive.followTrajectoryAsync(traj1);
                       currentState = State.TRAJECTORY_1;
                   }
                   break;


               case TRAJECTORY_1:
                   if(!drive.isBusy()) {
                       currentState = State.AUTO_TARGET_1;
                   }
                   break;


               case AUTO_TARGET_1:

                   autoTargeting = true;

                   if(robot.drive.scanForTag(4) || robot.drive.scanForTag(5) || robot.drive.scanForTag(6)) {
                       robot.drive.alignToTargetTag(AprilTags.spikeToAprilTagRED.get(spike))
                   }

                   if(MathUtil.isInRange(robot.drive.getScannedTag(AprilTags.spikeToAprilTagRED.get(spike)).ftcPose.range, 10)) {
                       robot.lifter.claw.setPosition(Constants.ClawConstants.clawOpen);

                       autoTargeting = false;
                       stateTimer.reset();
                       drive.followTrajectoryAsync(park);
                       currentState = State.PARK;
                   } else if (stateTimer.milliseconds() >= 10000) {
                     autoTargeting = false;
                     stateTimer.reset();
                     drive.followTrajectoryAsync(park)
                     currentState = State.PARK;
                   }

                   break;


               case PARK:

                   if(stateTimer.milliseconds() >= 250) {
                       robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);

                       if(stateTimer.milliseconds() >= 400) {
                           robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer)
                           robot.setLifterWristRoll(Constants.ClawConstants.wristRollVertical)
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
       }
   }
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
