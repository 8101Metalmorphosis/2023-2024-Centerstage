package org.firstinspires.ftc.teamcode;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggle;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TestTeleOp", group = "!")
public class TestTeleOp extends LinearOpMode {


    Servo drone;

    Robot robot;

    ButtonToggleAdvanced side;

    ButtonToggle FOD;
    ButtonToggle antiTip;
    ButtonToggle alignAprilTag;
    ButtonToggle closeClaw;

    Telemetry.Item allianceT = telemetry.addData("Alliance", "NONE");

    // GAMEPAD 1
    double LY1;
    double LX1;
    double RY1;
    double RX1;

    boolean A1;

    boolean START1;
    boolean BACK1;

    // GAMEPAD 2
    double LY2;
    double LX2;
    double RY2;
    double RX2;

    boolean A2;



    double clawPosition;



    //April tag init
    static final boolean useWebcam = true;

    // This processes the actual april tag, figures out which one it is,
    // and also gets other important values such as location and rotation from camera.
    private AprilTagProcessor aprilTag;

    // This is the actual viewport, used to stream the camera,
    // which the april tag processor uses to find april tag data.
    private VisionPortal visionPortal;




    double loopTimeAvg;

    @Override
    public void runOpMode() {

        drone = hardwareMap.get(Servo.class, "Drone");

        robot = new Robot(hardwareMap);


        initializeAprilTags();


        // RED == -1, BLUE == 1
        side = new ButtonToggleAdvanced(0);

        FOD = new ButtonToggle();
        antiTip = new ButtonToggle();
        alignAprilTag = new ButtonToggle();
        closeClaw = new ButtonToggle();

        ElapsedTime time = new ElapsedTime();

        double lastTime = 0;
        double[] loopTime = {};

        int currentLoop = 0;

        drone.setPosition(0);

        while (opModeInInit()) {

            updateTelemetry();

            isSwitchAlliance();


        }

        waitForStart();

        if(opModeIsActive()) {

            robot.imuYawOffset = robot.orientation.firstAngle;

            while(opModeIsActive()) {


                if(gamepad1.x) {
                    drone.setPosition(.5);
                }

                readInputs();
                robot.update();

                FOD.update(A1);
                antiTip.update(false);
                alignAprilTag.update(BACK1);

                // update april tag telemetry data.
                telemetryAprilTag();

                telemetry.update();

                robot.updateDrive(LY1, LX1, RX1, FOD.getState(), antiTip.getState(), alignAprilTag.getState(), side);

                closeClaw.update(A2);
                if (closeClaw.getState()){
                    robot.setClawPosition(Constants.Claw.close);
                } else {
                    robot.setClawPosition(Constants.Claw.open);
                }

                if(gamepad2.dpad_up) {
                    robot.setSlidePosition(1400);
                } else if (gamepad2.dpad_down) {
                    robot.setSlidePosition(-20);
                } else if (gamepad2.dpad_left) {
                    robot.setSlidePosition(200);
                }

                if(Math.abs(gamepad2.left_stick_y) > .05) {
                    robot.changeSlidePosition((int) MathUtil.putInRange(-20, (int) (robot.lifter.masterPos + (-gamepad2.left_stick_y * 600)), 1600));
                }

                if(gamepad1.y) {
                    robot.imuYawOffset = -robot.orientation.firstAngle;
                }



                isSwitchAlliance();
                telemetryAprilTag();
                updateTelemetry();



                // Calculate Loop Time
                if(loopTime.length == 2) {
                    loopTimeAvg = (loopTime[0] + loopTime[1] + loopTime[2]) / 3;
                    currentLoop = 0;
                } else {
                    loopTimeAvg = time.milliseconds() - lastTime;
                    lastTime = time.milliseconds();
                    currentLoop = currentLoop + 1;
                }
            }
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> detections = aprilTag.getDetections();
        robot.drive.currentDetections = detections;
//        telemetry.addData("# of AprilTags Detected", detections.size());


        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
//
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
        //telemetry.addData("Turn Power", robot.drive.turnPower);
//        telemetry.addData("Claw Position", robot.Claw.getPosition());
//        telemetry.addData("Test Motor Position", robot.lifter.masterSlide.getCurrentPosition());
//        telemetry.addData("Test Motor Target Position", robot.lifter.targetPos);
//
//        telemetry.addData("Test Motor Velocity", robot.lifter.masterSlide.getVelocity());



        telemetry.update();

    }

    public void readInputs() {
        // GAMEPAD 1
        LY1 = -gamepad1.left_stick_y;
        LX1 = gamepad1.left_stick_x;
        RY1 = gamepad1.right_stick_y;
        RX1 = gamepad1.right_stick_x;
        A1 = gamepad1.a;
        START1 = gamepad1.start;
        BACK1 = gamepad1.back;

        // GAMEPAD 2

        LY2 = gamepad2.left_stick_y;
        LX2 = gamepad2.left_stick_x;
        RY2 = gamepad2.right_stick_y;
        RX2 = gamepad2.right_stick_x;


        A2 = gamepad2.a;
    }

    public void isSwitchAlliance() {
        if(gamepad1.back && side.getState() == ButtonToggleAdvanced.NEUTRAL_STATE) {
            side.setState(-1);
        }

        side.update(gamepad1.left_bumper);

        if(side.getState() == ButtonToggleAdvanced.NEUTRAL_STATE) {
            allianceT.setValue("Alliance:", "NONE");
        } else if (side.getState() == ButtonToggleAdvanced.OFF_STATE) {
            allianceT.setValue("Alliance:", "RED");
        } else if (side.getState() == ButtonToggleAdvanced.ON_STATE) {
            allianceT.setValue("Alliance:", "BLUE");
        }
    }

    public void updateTelemetry() {

        // Driver / Gamepad 1 information

//        telemetry.addData("LY", LY1);
//        telemetry.addData("LX", LX1);
//        telemetry.addData("RY", RY1);
//        telemetry.addData("RX", RX1);
//
//        telemetry.addData("FOD", FOD.getState());
//        telemetry.addData("Anti-Tip", antiTip.getState());
//        telemetry.addData("Align to April Tag", alignAprilTag.getState());
//
//        telemetry.addLine();
//
//        // Manipulator / Gamepad 2 information
//        telemetry.addLine("Manipulator / Gamepad 2");
//        telemetry.addData("LY", LY2);
//        telemetry.addData("LX", LX2);
//        telemetry.addData("RY", RY2);
//        telemetry.addData("RX", RX2);
//        telemetry.addLine();
//
//
//        telemetry.addData("Target Power", robot.lifter.getPower());
//        telemetry.addData("Arm Position", robot.lifter.masterSlide.getCurrentPosition());
//        telemetry.addData("Target Arm Position", robot.lifter.targetPos);
//
//
//        telemetry.addLine();
//        telemetry.addData("YAW", Math.toDegrees(robot.getImuYaw()));
//        telemetry.addData("PITCH", Math.toDegrees(robot.orientation.secondAngle));
//        telemetry.addData("ROLL", Math.toDegrees(robot.orientation.thirdAngle));
//        telemetry.addData("Loop Time Average (3 Cycles)", loopTimeAvg);
//
//        telemetry.update();
    }

    private void initializeAprilTags() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // fx, fy, cx, cy.
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        setManualExposure(6, 90);

        if(useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(800, 448));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);


        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
