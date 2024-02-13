package org.firstinspires.ftc.teamcode.Subsystems;


import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.FTCutil.PID.PIDController;
import org.firstinspires.ftc.teamcode.OLD.NikoRunner.library.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class MecanumDrive {

    PIDController headingController = new PIDController(1, 0, 0, 0);


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public List<AprilTagDetection> currentDetections;


    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx BackLeft;
    DcMotorEx BackRight;

    public double frontLeftPos;
    public double frontRightPos;
    public double backLeftPos;
    public double backRightPos;


    public MecanumDrive(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");


        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initAprilTag(hardwareMap);
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    public void update() {
        frontLeftPos = FrontLeft.getCurrentPosition();
        frontRightPos = FrontRight.getCurrentPosition();
        backLeftPos = BackLeft.getCurrentPosition();
        backRightPos = BackRight.getCurrentPosition();
    }

    public void drive(double LY, double LX, double RX, boolean FOD, boolean antiTip, boolean alignAprilTag,
                      ButtonToggleAdvanced side, double yaw, double pitch, double roll) {

        if (alignAprilTag) {

            currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == 5) {
                        alignToTag(detection);
                    }
                } else {
                    FrontLeft.setPower(0);
                    FrontRight.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                }
            }

            if(currentDetections.size() == 0) {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }
            return;
        }

        if (FOD) {
            Vector2d rotatedValues = MathUtil.rotateByAngle(new Vector2d(LX, LY), -yaw);

            LX = rotatedValues.getX();
            LY = rotatedValues.getY();
        }

        FrontLeft.setPower((LY + LX + RX));
        FrontRight.setPower((LY - LX - RX));
        BackLeft.setPower((LY - LX + RX));
        BackRight.setPower((LY + LX - RX));
    }

    public boolean findTargetTag(int targetTagID) {
        currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == targetTagID) {
                    alignToTag(detection);
                }
            } else {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }
        }

        if(currentDetections.size() == 0) {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

        return false;
    }

    public AprilTagDetection getScannedTag(int targetTagID) {
        currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == targetTagID) {
                    return detection;
                }
            }
        }

        if(currentDetections.size() == 0) {
            return null;
        }
    }

    public void alignToTag(AprilTagDetection detection) {
        double targetDistance = 9;

        double DISTANCE_GAIN = 0.036;
        double STRAFE_GAIN = 0.02;
        double TURN_GAIN = 0.025;

        double MAX_DRIVE_SPEED = .4;
        double MAX_STRAFE_SPEED = .2;
        double MAX_TURN_SPEED = .1;

        // DRIVE
        double rangeError = detection.ftcPose.range - targetDistance;
        double rangeValue = Range.clip(rangeError * DISTANCE_GAIN, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        // STRAFE
        double yawError = detection.ftcPose.yaw;
        double yawValue = Range.clip(-yawError * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

        // TURN
        double headingError = detection.ftcPose.bearing;
        double headingValue = Range.clip(headingError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        FrontLeft.setPower(-rangeValue + yawValue - headingValue);
        FrontRight.setPower(-rangeValue - yawValue + headingValue);
        BackLeft.setPower(-rangeValue - yawValue - headingValue);
        BackRight.setPower(-rangeValue + yawValue + headingValue);
    }

    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public void setManualExposure(int exposureMS, int gain) {
        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
