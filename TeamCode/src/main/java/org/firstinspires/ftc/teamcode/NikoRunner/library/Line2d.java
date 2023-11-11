package org.firstinspires.ftc.teamcode.NikoRunner.library;

import java.util.ArrayList;

public class Line2d {
    ArrayList<Vector2d> pointList;
    Rotation2d targetRotation;
    String type;

    public Line2d(ArrayList<Vector2d> pointList, Rotation2d targetRotation, String type) {
        this.pointList = pointList;
        this.targetRotation = targetRotation;
        this.type = type;
    }

    public Pose2d getEndPose() {
        return new Pose2d(pointList.get(pointList.size()), targetRotation);
    }

    public Rotation2d getTargetRotation2d() {
        return targetRotation;
    }

    public String getType() {
        return type;
    }
}
