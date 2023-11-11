package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    Mat mat = new Mat();

    Mat whiteMat = new Mat();
    Mat purpleMat = new Mat();
    Mat yellowMat = new Mat();
    Mat greenMat = new Mat();


    ArrayList<MatOfPoint> whiteContours = new ArrayList<>();
    ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
    ArrayList<MatOfPoint> greenContours = new ArrayList<>();


    public double lowArea = 1000;
    public double maxArea = 2500;

    Mat hierarchy = new Mat();


    public PixelPipeline(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {

        Mat output = input;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar[] whiteScalar = {new Scalar(0, 0, 0), new Scalar(0, 0, 0)};
//        Scalar[] whiteScalar = {new Scalar(0, 0, 83), new Scalar(180, 38, 255)};
        Scalar[] purpleScalar = {new Scalar(120, 60, 75), new Scalar(150, 210, 255)};
        Scalar[] yellowScalar = {new Scalar(10, 158, 78), new Scalar(32, 255, 195)};
        Scalar[] greenScalar = {new Scalar(28, 85, 0), new Scalar(64, 255, 255)};




        Imgproc.GaussianBlur(mat, mat, new Size(5.0, 15.0), 0.00);

        Core.inRange(mat, whiteScalar[0], whiteScalar[1], whiteMat);
        Core.inRange(mat, purpleScalar[0], purpleScalar[1], purpleMat);
        Core.inRange(mat, yellowScalar[0], yellowScalar[1], yellowMat);
        Core.inRange(mat, greenScalar[0], greenScalar[1], greenMat);


        double whiteValue = Core.sumElems(whiteMat).val[0] / 255;
        double purpleValue = Core.sumElems(purpleMat).val[0] / 255;
        double yellowValue = Core.sumElems(yellowMat).val[0] / 255;
        double greenValue = Core.sumElems(greenMat).val[0] / 255;


        visualizeContours(whiteMat, whiteContours, new Scalar(140, 100, 200), output);
        visualizeContours(purpleMat, purpleContours, new Scalar(140, 100, 200), output);
        visualizeContours(yellowMat, yellowContours, new Scalar(140, 100, 200), output);
        visualizeContours(greenMat, greenContours, new Scalar(140, 100, 200), output);

        telemetry.addData("White Value", whiteValue);
        telemetry.addData("Purple Value", purpleValue);
        telemetry.addData("Yellow Value", yellowValue);
        telemetry.addData("Green Value", greenValue);
        telemetry.update();

        // mat.release();
        whiteMat.release();
        purpleMat.release();
        yellowMat.release();
        greenMat.release();



        return output;
    }

    private void visualizeContours(Mat input, ArrayList<MatOfPoint> contours, Scalar color, Mat output) {

        contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));

            if(rect.width * rect.height > lowArea && rect.width * rect.height < maxArea) {
//                Imgproc.drawContours(output, contours, -1, color);

                Imgproc.rectangle(output, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(140, 100, 200));
                System.out.println(contours.get(i));
            }
        }
    }
}