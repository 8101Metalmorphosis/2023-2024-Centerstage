package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RandomizationPixelPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    Mat output = new Mat();



    public RandomizationPixelPipeline(Telemetry t) {t = telemetry;}


//    static final Rect Screen = new Rect(
//        new Point(0, 0),
//        new Point(640, 480)
//    );

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

        Scalar RedScalarL = new Scalar(0, 0, 0);
        Scalar RedScalarH = new Scalar(180, 255, 255);
        Scalar[] BlueScalar = {new Scalar(99, 64, 133), new Scalar(147, 255, 236)};


        Core.inRange(output, RedScalarL, RedScalarH, output);


//        Mat red = output.submat(Screen);
//
//        telemetry.addData("Raw Red Value", (int) Core.sumElems(red).val[0]);

        Imgproc.rectangle(output, new Point(100, 50), new Point(300, 350), new Scalar(255, 255, 255), 2);

        return output;
    }
}
