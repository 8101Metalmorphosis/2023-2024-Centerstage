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

@Autonomous(name = "Red Audience Autonomous ASYNC", group = "! a")
public class RedAudienceAutonomousAsyncTest extends LinearOpMode {

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

        Trajectory rightSpike = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-42, -34))
                .build();


        }
    }
}
