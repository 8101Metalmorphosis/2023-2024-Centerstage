package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

import java.util.ArrayList;

public class Spline2d {
    ArrayList<Vector2d> pointList;
    Rotation2d targetRotation;
    String type;

    public Spline2d(ArrayList<Vector2d> pointList, Rotation2d targetRotation, String type) {
        this.pointList = pointList;
        this.targetRotation = targetRotation;
        this.type = type;
    }

    public ArrayList<Vector2d> getPoints() {
        return pointList;
    }

    public Pose2d getEndPose() {
        return new Pose2d(pointList.get(pointList.size()), targetRotation);
    }

    public Rotation2d getEndRotation() {
        return targetRotation;
    }
    
}