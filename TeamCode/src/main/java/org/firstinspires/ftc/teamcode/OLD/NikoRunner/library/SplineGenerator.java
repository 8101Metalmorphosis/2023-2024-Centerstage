package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

import java.util.ArrayList;

public class SplineGenerator {
    double waypoints = 500; // SHOULD BE 6-12 WHEN IN USE
    double lerpMultiplier = 1 / waypoints;

    final double defaultTanDistance = 5;


    ArrayList<Vector2d> splinePoints = new ArrayList<>();
    Spline2d spline;

    SplineGenerator(Pose2d startPose, Rotation2d startTangent, 
        Pose2d endPose, Rotation2d endTangent, double endTanDistance, String type) {

        if (endTanDistance == 0) {
            endTanDistance = defaultTanDistance;
        }
    
        Vector2d endTangentVector = calculateTangentVector(endPose.getVector(), endTangent, endTanDistance);

        if (startTangent == null) {
            startTangent = calculateTangentRotation(startPose.getVector(), endTangentVector);
        }
        Vector2d startTangentVector = calculateTangentVector(startPose.getVector(), startTangent, endTanDistance);

        lerp(startPose.getVector(), startTangentVector, endPose.getVector(), endTangentVector);

        spline = new Spline2d(splinePoints, endPose.getRotation2d(), type);
    }

    public ArrayList<Vector2d> lerp(Vector2d startVector, Vector2d startTangent,
        Vector2d endVector, Vector2d endTangent) {

        lerpMultiplier = 1 / waypoints;

        int n = 0;
        for (double t = lerpMultiplier; !(t > 1); t = t + lerpMultiplier) {
            splinePoints.add(calculateSplinePoints(startVector, startTangent, endTangent, endVector, t));
            System.out.print("(" + splinePoints.get(n).getX() + ",");
            System.out.print(splinePoints.get(n).getY() + ")" + ",");
            n++;
        }

        return splinePoints;
    }

    public Spline2d getSpline() {
        return spline;
    }

    public Vector2d calculateSplinePoints(
        Vector2d startVector, Vector2d startTangent, Vector2d endTangent, Vector2d endVector, double t) {
        
        return new Vector2d(Round.roundToDecimal(
            ((Math.pow(1 - t, 3) * startVector.getX()) +
                    (3 * Math.pow(1 - t, 2) * t * startTangent.getX()) +
                    ((3 * (1 - t) * Math.pow(t, 2) * endTangent.getX())) +
                    (Math.pow(t, 3) * endVector.getX())),
            3),
            Round.roundToDecimal(((Math.pow(1 - t, 3) * startVector.getY()) +
                    (3 * Math.pow(1 - t, 2) * t * startTangent.getY()) +
                    ((3 * (1 - t) * Math.pow(t, 2) * endTangent.getY())) +
                    (Math.pow(t, 3) * endVector.getY())), 3));
    }

    
    /**
     * Translates a tangents rotation into a vector
     * 
     * @param initialVector
     * @param rotation
     * @param distance
     * @return Returns tangent rotation as a point using an inital vector.
     */
    public Vector2d calculateTangentVector(Vector2d initialVector, Rotation2d rotation, double distance) {
        Vector2d tangentVector = new Vector2d(
                Math.sin(rotation.getRotation()) * distance,
                Math.cos(rotation.getRotation()) * distance);
        Vector2d updatedVector = new Vector2d(
                initialVector.getX() + tangentVector.getX(),
                initialVector.getY() + tangentVector.getY());
        return updatedVector;
    }

    /**
     * Translates a Vector2d to a Rotation2d from another point.
     * @param initialVector
     * @param tangentVector
     * @return Returns
     */
    public Rotation2d calculateTangentRotation(Vector2d initialVector, Vector2d tangentVector) {
        double xDisp = tangentVector.getX() - initialVector.getX();
        double yDisp = tangentVector.getY() - initialVector.getY();

        Rotation2d tangentRotation = new Rotation2d(Math.atan2(xDisp, yDisp));
        return tangentRotation;
    }
}
