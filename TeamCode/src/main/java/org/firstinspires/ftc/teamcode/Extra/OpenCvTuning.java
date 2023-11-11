//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//
//
//@TeleOp(name = "OpenCv Tuning", group = "! Tuning")
//public class OpenCvTuning extends LinearOpMode {
//
//
//    OpenCvCamera phoneCam;
//
//    public enum Selected {
//        HUE, SATURATION, VALUE
//    }
//
//    public int selectedAsInt = 0;
//    public Selected selected = Selected[selectedAsInt];
//
//    public ButtonToggle showContours = new ButtonToggle();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        int cameraMonitorViewId = hardwareMap.appContext
//        .getResources().getIdentifier("cameraMonitorViewId",
//                "id", hardwareMap.appContext.getPackageName());
//
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//
//        OpenCvColorTuner detector = new OpenCvTuningAutonomous(telemetry);
//        webcam.setPipeline(detector);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        webcam.startStreaming();
//
//        waitForStart();
//
//        while(opModeIsActive) {
//
//            if(gamepad1.dpad_up) {
//                int targetValue;
//                if(selected = Selected.HUE) {
//                    targetValue = 0;
//                } else if (selected = Selected.SATURATION) {
//                    targetValue = 1;
//                } else {
//                    targetValue = 2;
//                }
//                detector.LOW_COLOR_RANGE[targetValue] = detector.LOW_COLOR_RANGE + 1;
//            } else if (gamepad1.dpad_down) {
//                detector.LOW_COLOR_RANGE[targetValue] = detector.LOW_COLOR_RANGE - 1;
//            }
//
//            if(gamepad1.x) {
//                selectedAsInt = 0;
//                selected = Selected[selectedAsInt];
//            }
//
//            if(gamepad1.y) {
//                selectedAsInt = 1;
//                selected = Selected[selectedAsInt];
//            }
//
//            if(gamepad1.b) {
//                selectedAsInt = 1;
//                selected = Selected[selectedAsInt];
//            }
//
//
//
//
//
//        }
//    }
//}
