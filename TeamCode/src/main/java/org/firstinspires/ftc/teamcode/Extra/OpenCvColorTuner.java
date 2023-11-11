//package org.firstinspires.ftc.teamcode.Extra;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggle;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class OpenCvColorTuner extends OpenCvPipeline {
//
//    Telemetry telemetry;
//
//    Mat mat = new Mat();
//
//    Mat targetMat = new Mat();
//
//
//    ArrayList<MatOfPoint> targetContours = new ArrayList<>();
//
//    public final int HSV_COLOR_SPACE = 1;
//    public final int YCRBR_COLOR_SPACE = 2;
//
//    public int COLOR_SPACE = HSV_COLOR_SPACE;
//
//    public int lowArea = 100;
//
//    public int[] LOW_COLOR_RANGE = {0, 0, 0};
//    public int[] HIGH_COLOR_RANGE = {180, 255, 255};
//
//    public ButtonToggle seeContours = new ButtonToggle();
//
//
//
//    Mat hierarchy = new Mat();
//
//
//    public OpenCvColorTuner(Telemetry t) {telemetry = t;}
//
//    @Override
//    public Mat processFrame(Mat input) {
//
//        Mat output = input;
//
//        if(HSV[0] > 180) {
//            HSV[0] = 180;
//        } else if (HSV[0] < 0) {
//            HSV[0] = 0;
//        }
//
//        if(HSV[1] > 255) {
//            HSV[1] = 255;
//        } else if (HSV[1] < 0) {
//            HSV[1] = 0;
//        }
//
//        if(HSV[2] > 255) {
//            HSV[0] = 255;
//        } else if (HSV[2] < 0) {
//            HSV[2] = 0;
//        }
//
//
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//
//        Scalar[] targetScalar = {
//            new Scalar(LOW_COLOR_RANGE[0], LOW_COLOR_RANGE[1], LOW_COLOR_RANGE[2]),
//            new Scalar(HIGH_COLOR_RANGE[0], HIGH_COLOR_RANGE[1], HIGH_COLOR_RANGE[2])
//        };
//
//
//
//        Imgproc.GaussianBlur(mat, mat, new Size(5.0, 15.0), 0.00);
//
//        Core.inRange(mat, targetScalar[0], targetScalar[1], targetMat);
//
//
//        double targetValue = Core.sumElems(targetMat).val[0] / 255;
//
//        if(seeContours) {
//            visualizeContours(targetMat, targetContours, low, new Scalar(140, 100, 200), output);
//        }
//
//
//        telemetry.addData("Target Value", targetValue);
//
//        telemetry.addLine();
//        telemetry.addLine("COLOR RANGES");
//        telemetry.addData(" Hue", LOW_COLOR_RANGE[0] + " - " + HIGH_COLOR_RANGE[0]);
//        telemetry.addData(" Saturation", LOW_COLOR_RANGE[1] + " - " + HIGH_COLOR_RANGE[1]);
//        telemetry.addData(" Value", LOW_COLOR_RANGE[2] + " - " + HIGH_COLOR_RANGE[2]);
//        telemetry.update();
//
//
//        targetMat.release();
//
//
//        return output;
//    }
//
//    private void visualizeContours(Mat input, ArrayList<MatOfPoint> contours, Scalar color, Mat output) {
//
//        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        for (int i = 0; i < contours.size(); i++) {
//            Rect rect = Imgproc.boundingRect(contours.get(i));
//
//            if(rect.area <= lowArea) {
//                Imgproc.drawContours(output, contours, -1, color);
//
//                Imgproc.rectangle(output, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(140, 100, 200));
//            }
//        }
//    }
//}