package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    Mat output = new Mat();


    double targetPercent = .06;

    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public Location location;


    public RedPipeline(Telemetry t) {telemetry = t;}


    static final Rect LEFT = new Rect(
            new Point(300, 100),
            new Point(560, 325)
    );

    static final Rect MIDDLE = new Rect(
        new Point(0, 100),
        new Point(280, 350)
    );

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

        Scalar[] RedScalar = {new Scalar(170, 100, 30), new Scalar(180, 255, 255)};
//        Scalar[] BlueScalar = {new Scalar(99, 64, 133), new Scalar(147, 255, 236)};


        Core.inRange(output, RedScalar[0], RedScalar[1], output);
//        Core.inRange(output, BlueScalar[0], BlueScalar[1], output);

        Mat middleMat = output.submat(MIDDLE);
        Mat leftMat = output.submat(LEFT);

        double middleValue = Core.sumElems(middleMat).val[0] / MIDDLE.area() / 255;
        double leftValue = Core.sumElems(leftMat).val[0] / LEFT.area() / 255;

        telemetry.addData("MIDDLE percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("LEFT percentage", Math.round(leftValue * 100) + "%");
        telemetry.update();

        if(middleValue > targetPercent && leftValue > targetPercent) {
            if (middleValue > leftValue) {
                location = Location.MIDDLE;
            } else {
                location = Location.LEFT;
            }
        } else if (middleValue > targetPercent) {
            location = Location.MIDDLE;
        } else if (leftValue > targetPercent) {
            location = Location.LEFT;
        } else {
            location = Location.RIGHT;
        }

        Imgproc.rectangle(output, LEFT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(output, MIDDLE, new Scalar(125, 0, 0), 2);

        Imgproc.rectangle(input, LEFT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, MIDDLE, new Scalar(125, 0, 0), 2);

        middleMat.release();
        leftMat.release();

        return input;
    }

    public Location getLocation() {
        return location;
    }
}
