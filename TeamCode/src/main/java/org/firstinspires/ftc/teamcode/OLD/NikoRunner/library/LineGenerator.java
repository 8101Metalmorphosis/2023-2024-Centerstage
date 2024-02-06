package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

import java.util.ArrayList;

public class LineGenerator {
    double waypoints = 200;
    double lerpMultiplier = 1 / waypoints;

    ArrayList<Vector2d> linePoints = new ArrayList<>();
    Line2d line;

    LineGenerator(Pose2d startPose, Pose2d endPose, String type) {
        lerpMultiplier = 1 / waypoints;

        int n = 0;
        for (double t = lerpMultiplier; !(t > 1); t = t + lerpMultiplier) {
            linePoints.add(calculatePoints(startPose, endPose, t));
            System.out.print("(" + linePoints.get(n).getX() + ",");
            System.out.print(linePoints.get(n).getY() + ")" + ",");
            n++;
        }

        line = new Line2d(linePoints, endPose.getRotation2d(), type);
    }

    public Line2d getLine() {
        return line;
    }

    public Spline2d getAsSpline() {
        return new Spline2d(linePoints, line.getTargetRotation2d(), line.getType());
    }

    public Vector2d calculatePoints(Pose2d startPose, Pose2d endPose, double t) {
        return new Vector2d(
            Round.roundToDecimal(startPose.getX() + ((1-t) * (endPose.getX() - startPose.getX())), 3), 
            Round.roundToDecimal(startPose.getY() + ((1-t) * (endPose.getY() - startPose.getY())), 3));
    }
}
