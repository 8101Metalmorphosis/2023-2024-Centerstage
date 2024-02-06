package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

public class SplineValues2d {
    Pose2d endPose;
    Rotation2d endTangent;

    double endTanDistance;

    String type;


    public SplineValues2d(Pose2d endPose, Rotation2d endTangent, double endTanDistance, String type) {
        this.endPose = endPose;
        this.endTangent = endTangent;
        this.endTanDistance = endTanDistance;
        this.type = type;
    }

    public Pose2d getPose() {
        return endPose;
    }

        // Returned values

    /**
     * @return returns this Rotation2d as a Rotation2d.
     */
    public Rotation2d getRotation2d() {
        return endTangent.getRotation2d();
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in radians.
     */
    public double getRotation() {
        return endTangent.getRotation();
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in degrees.
     */
    public double getRotationDEG() {
        return endTangent.getRotationDEG();
    }

    public double getEndTanDistance() {
        return endTanDistance;
    }

    public String getType() {
        return type;
    }


    // Set Values

    /** Updates this Rotation2ds rotation.
    * @param rotation value in radians.
    */
    public void setRotation(double rotation) {
        this.endTangent.setRotation(rotation);
    }

    
}
