package org.firstinspires.ftc.teamcode.FTCutil;

import org.firstinspires.ftc.teamcode.OLD.NikoRunner.library.Vector2d;

public class MathUtil {

    public static double putInRange(double min, double value, double max){
        if(value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    public static boolean isInRange(double min, double value, double max){
        return min < value && value < max? true : false;
    }

    public static boolean isInRange(double value, double range) {
        return -range < value && value < range? true : false;
    }

    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public static Vector2d rotateByAngle(Vector2d vector, double radians) {
        double xValue = vector.getX() * Math.cos(radians) - vector.getY() * Math.sin(radians);
        double yValue = vector.getX() * Math.sin(radians) + vector.getY() * Math.cos(radians);

        return new Vector2d(xValue, yValue);
    }

    public static boolean in2DimensionalRange(Vector2d vector1, Vector2d vector2, double maxDistance) {
        Vector2d distance = new Vector2d(
            vector2.getX() - vector1.getX(),
            vector2.getY() - vector1.getY()
        );

        if(distance.getX() <= maxDistance || distance.getY() <= maxDistance) {
            return true;
        }

        return false;
    }


    public static int calculateTimeMS(double lastPosition, double targetPosition, int timePerHalf) {
        return (int) Math.abs(((targetPosition - lastPosition) / .5) * timePerHalf);
    }
}
