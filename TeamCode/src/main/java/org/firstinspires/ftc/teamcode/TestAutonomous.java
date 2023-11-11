package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Test Autonomous", group = "!")
public class TestAutonomous extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {


//        int cameraMonitorViewId = hardwareMap.appContext
//        .getResources().getIdentifier("cameraMonitorViewId",
//                "id", hardwareMap.appContext.getPackageName());
//
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//
//        PixelPipeline detector = new PixelPipeline(telemetry);
//        webcam.setPipeline(detector);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });

        waitForStart();


    }

//    public Action testMotorSetPowerAction() {
//
//
//

//
//    }
}
