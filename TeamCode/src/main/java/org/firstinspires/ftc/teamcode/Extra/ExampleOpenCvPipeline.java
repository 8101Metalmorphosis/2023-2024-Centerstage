package org.firstinspires.ftc.teamcode.Extra;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ExampleOpenCvPipeline extends OpenCvPipeline {

    Telemetry telemetry;


    // This matrix we will use as a general HSV image matrix
    Mat mat = new Mat();

    // This matrix we will use to store the purple values in the image
    Mat purpleMat = new Mat();


    /*
       Since contours only work well when the image is gray scale, we will look for the white in the image (the color we want to sense will be white),
       This list will be used to store the edges of the white area in the grayscale image (color we sensed)
       this will make sense when you go into processFrame
     */
    ArrayList<MatOfPoint> purpleContours = new ArrayList<>();

    // you will see later
    Mat hierarchy = new Mat();

    // this is what runs when you create a new ExampleOpenCvPipeline,
    // and example would be ExampleOpenCvPipeline examplePipeline = new ExampleOpenCvPiepline(telemetry);
    public ExampleOpenCvPipeline(Telemetry t) {telemetry = t;}


    // This is the main thing that will actually run and get data for you
    @Override
    public Mat processFrame(Mat input) {

        // We want the image to output as a normal camera image so we will originally set the output image matrix to the input
        Mat output = input;

        // We convert the color format from RGB to HSV as it is easier to make a color range using HSV than RGB.
        // We take in the input matrix and turn it into HSV, which we will store into our mat matrix.
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // A Scalar is variable that stores 3 values (x, y, z basically)
        // We will use these for our color range we want to sense
        // these are the scalar values we use to sense the purple pixel, (check next comment)
        Scalar[] purpleScalar = {new Scalar(110, 50, 0), new Scalar(166, 255, 255)};

        // purpleL will be the low range, so all the values have to be lower than the purpleH values, which are the higher values.
        // HSV is Hue, Saturation, Value (value is how bright so black would be 0, white would be 255), so we put Hue in x, Saturation in y, and Value in z.
        // We need a color range instead of a set value because the light hitting the object,
        // the pixel, will change the color the camera will see, so this makes it way more accurate.
        Scalar purpleL = new Scalar(110, 50, 0);
        Scalar purpleH = new Scalar(166, 255, 255);

        // This will get the colors inside the range we set above, so it will look for pixels with a Hue range of 110-166, Saturation of 50-255, and Value of 0-255.
        // Once it gets these pixels, it will cast it onto another mat, our purpleMat,
        // which will store which pixels are in the range and which ar not (the pixels in that range will be colored white, and the ones that are not will be black.
        Core.inRange(mat, purpleScalar[0], purpleScalar[1], purpleMat);

        // This isn't very important, but we can use this to see how much of an area has the purple color.
        // In Centerstage, we could set 3 areas on our camera we want to sense, the 3 lines at the beginning of auto, and find which area has the most of that color
        // in this case, purple, and we could use that to know what area has the most of a specific color. Which cause theres a random chance its on 1 of 3 spots
        // we can figure out which spot most likely has the color we want to sense
        double purpleValue = Core.sumElems(purpleMat).val[0] / 255;

        // This function will find contours (edge of the white/color we want to sense) on our purple mat, and then use those pixel coordinates and draw an outline
        // of the purple, and also a box that goes over the center of the object so we can figure out the center pixel coordinate of the object.
        visualizeContours(purpleMat, purpleContours, new Scalar(140, 100, 200), output);


        telemetry.addData("Purple Value", purpleValue);
        telemetry.update();


        return output;
    }

    private void visualizeContours(Mat input, ArrayList<MatOfPoint> contours, Scalar color, Mat output) {

        // This finds the contours on the purpleMat matrix, so the edge of the purple.
        // RETR_EXTERNAL means it only covers the outside of the object, so when we sense pixels,
        // it wont have a hole in the middle (I will send image, if I forget tell me)
        // I don't really know what Chain_Approx_Simple does, but im pretty sure it connects the pixel coordinates from the contours,
        // so the outline of the purple is lines not just points, if we had Chain_Approx_None, im guessing it will only use points, no lines.
        // the contours variable is the ArrayList we made earlier, which will store the contours we find.
        // input is the matrix we want to sense the contours from
        // hierarchy means it will sense the white pixels from the top down (pretty sure)
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // This draws the contours we find on the matrix image, onto the output image
        // contourIDX (-1) means we draw all contours, if we wanted to draw a specific one, we could put a value 1 - max value
        Imgproc.drawContours(output, contours, -1, color);

        for (int i = 0; i < contours.size(); i++) {
            // this gets us the top left coordinate of the contour we find, and also the width and height, so we can make an actual box around it.
            Rect rect = Imgproc.boundingRect(contours.get(i));

            // Create rectangle around sensed color
            // The scalar is the color of the box
            Imgproc.rectangle(
                    output,
                    new Point(rect.x, rect.y),
                    new Point(rect.x + rect.width, rect.y + rect.height),
                    new Scalar(140, 100, 200),
                    1,
                    1);
        }
    }
}
