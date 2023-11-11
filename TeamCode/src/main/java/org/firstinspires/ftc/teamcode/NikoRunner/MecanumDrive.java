package org.firstinspires.ftc.teamcode.NikoRunner;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.FTCutil.PID.PIDController;
import org.firstinspires.ftc.teamcode.FTCutil.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Spline2d;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Trajectory;
import org.firstinspires.ftc.teamcode.NikoRunner.library.Vector2d;

public class MecanumDrive {

    public HardwareMap hardwareMap;

    public DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;
    public BNO055IMU imu;

    // PID controllers (tune all values)
//    public PIDController xController = new PIDController(0, 0, 0);
//    public PIDController yController = new PIDController(0, 0, 0);
//
//    public PIDController thetaController = new PIDController(0, 0, 0);

    // This sets the acceptance range between the robot and the target position when following trajectories. (in Inches)
    public double acceptedError = 1;


    public TwoWheelLocalizer localizer;

    // Set these values if using dead wheels to localize.
    double countPerRev = 8192;
    double wheelRadius = 1.37795 / 2;

    double inPerTicks = countPerRev / (2 * Math.PI * wheelRadius);


    public MecanumDrive(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        // Set to the name of the hardware.
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set motor directions


    }

    public void update() {
        localizer.update();
    }

    public void followTrajectory(Trajectory traj) {
        ArrayList<Spline2d> path = traj.getPoints();

        for (int currentPath = 1; currentPath < path.size(); currentPath++) {
            ArrayList<Vector2d> points = path.get(currentPath).getPoints();

            for (int currentPoint = 0; currentPoint < points.size(); currentPoint++) {

                // Most likely wrong, will also switch to profiled PID controller so it isnt jerky, and so its easier to control the velocity
                
                // Using while True temporarily so we can test.
                boolean a = true;
                while(a) {
                    Vector2d targetVector = 
                        new Vector2d(
                            points.get(currentPoint).getX(),
                            points.get(currentPoint).getY()
                    );

                    int targetTheta =
                            (int) path.get(currentPath).getEndRotation().getRotation();
                

//                    double xPower =
//                        xController.update((int) localizer.getPose().getX(), (int) targetVector.getX());
//                    double yPower =
//                        yController.update((int) localizer.getPose().getY(), (int) targetVector.getY());
//                    double thetaPower =
//                        thetaController.update(0, targetTheta);

//                    FrontLeft.setPower(xPower + yPower + thetaPower);
//                    FrontRight.setPower(xPower - yPower - thetaPower);
//                    BackLeft.setPower(xPower - yPower + thetaPower);
//                    BackRight.setPower(xPower + yPower - thetaPower);
//
//
//                    if(MathUtil.in2DimensionalRange(localizer.getPose().getVector(), targetVector, acceptedError)) {
//                        break;
//                    }
                }
                
            }
        }
    }
}