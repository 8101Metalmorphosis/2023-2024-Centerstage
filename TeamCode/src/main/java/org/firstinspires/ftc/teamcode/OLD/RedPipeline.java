package org.firstinspires.ftc.teamcode.OLD;

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
        RIGHT,
        MIDDLE,
        LEFT
    }

    public Location location;


    public RedPipeline(Telemetry t) {telemetry = t;}


    static final Rect RIGHT = new Rect(
            new Point(420, 380),
            new Point(640, 480)
    );

    static final Rect MIDDLE = new Rect(
        new Point(120, 350),
        new Point(340, 480)
    );

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

        Scalar[] RedScalar = {new Scalar(170, 100, 30), new Scalar(180, 255, 255)};
//        Scalar[] BlueScalar = {new Scalar(99, 64, 133), new Scalar(147, 255, 236)};


        Core.inRange(output, RedScalar[0], RedScalar[1], output);
//        Core.inRange(output, BlueScalar[0], BlueScalar[1], output);

        Mat middleMat = output.submat(MIDDLE);
        Mat rightMat = output.submat(RIGHT);

        double middleValue = Core.sumElems(middleMat).val[0] / MIDDLE.area() / 255;
        double rightValue = Core.sumElems(rightMat).val[0] / RIGHT.area() / 255;

        telemetry.addData("MIDDLE percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("RIGHT percentage", Math.round(rightValue * 100) + "%");
        telemetry.update();

        if(middleValue > targetPercent && rightValue > targetPercent) {
            if (middleValue > rightValue) {
                location = Location.MIDDLE;
            } else {
                location = Location.RIGHT;
            }
        } else if (middleValue > targetPercent) {
            location = Location.MIDDLE;
        } else if (rightValue > targetPercent) {
            location = Location.RIGHT;
        } else {
            location = Location.LEFT;
        }

        Imgproc.rectangle(output, RIGHT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(output, MIDDLE, new Scalar(125, 0, 0), 2);

        Imgproc.rectangle(input, RIGHT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, MIDDLE, new Scalar(125, 0, 0), 2);

        middleMat.release();
        rightMat.release();

        return output;
    }

    public Location getLocation() {
        return location;
    }
}
