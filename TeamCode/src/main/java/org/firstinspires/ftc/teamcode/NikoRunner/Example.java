package org.firstinspires.ftc.teamcode.NikoRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NikoRunner.library.Pose2d;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Rotation2d;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Trajectory;

//@Disabled
@Autonomous(name = "Nikorunner Test", group = "")
public class Example extends LinearOpMode {

    public MecanumDrive drive;

    
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap);
        
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        

        Trajectory traj1 = new Trajectory.TrajectoryBuilder(startPose)
            .splineToProfiledHeading(new Pose2d(5, 0, 0), new Rotation2d(Math.toRadians(-90)), 1)
            .splineToProfiledHeading(new Pose2d(15, 10, 0), new Rotation2d(Math.toRadians(-90)), 8)
            .splineToProfiledHeading(new Pose2d(20, 10, 0), new Rotation2d(Math.toRadians(-90)), 1)
            .lineToProfiledHeading(new Pose2d(20, 20, 0))
            .build();

        waitForStart();

        drive.followTrajectory(traj1);
    }
}
