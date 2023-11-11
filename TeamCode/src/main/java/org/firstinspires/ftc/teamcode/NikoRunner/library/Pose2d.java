package org.firstinspires.ftc.teamcode.NikoRunner.library;

public class Pose2d {
    Vector2d poseVector;
    Rotation2d poseRotation;

    /** Creates a new Pose2d class.
     * @param x the x value for this Pose2d.
     * @param y the y value for this Pose2d.
     * @param rotation the rotation for this Pose2d.
     */
    public Pose2d(double x, double y, double rotation) {
        this.poseVector = new Vector2d(x, y);
        this.poseRotation = new Rotation2d(rotation);
    }

    /** Creates a new Pose2d class.
     * @param vector the x value for this Pose2d.
     * @param rotation the rotation for this Pose2d.
     */
    public Pose2d(Vector2d vector, double rotation) {
        this.poseVector = vector;
        this.poseRotation = new Rotation2d(rotation);
    }
    public Pose2d(Vector2d vector, Rotation2d rotation) {
        this.poseVector = vector;
     
        this.poseRotation = rotation;
    }
    public Pose2d(Vector2d vector) {
        this.poseVector = vector;
        this.poseRotation = null;
    }

// Vector Values

    // Returned values

    /**
     * @return returns as a vector.
     */
    public Vector2d getVector() {
        return poseVector.getVector();
    }

    /**
     * @return returns this vectors x value.
     */
    public double getX() {
        return poseVector.getX();
    }

    /**
     * @return returns this vectors y value.
     */
    public double getY() {
        return poseVector.getY();
    }


    // Set values

    /** Updates this Vector2ds x and y coordinates.
    * @param x value.
    * @param y value.
    */
    public void setVector(double x, double y) {
        poseVector.setX(x);
        poseVector.setY(y);
    }

    /**
     * @param x the new x value of this Pose2d.
     */
    public void setX(double x) {
        poseVector.setX(x);
    }
    
    /**
     * @param x the new y value of this Pose2d.
     */
    public void setY(double y) {
        poseVector.setY(y);
    }

// Rotation Values

// Returned values

    /**
     * @return returns this Rotation2d as a Rotation2d.
     */
    public Rotation2d getRotation2d() {
        return poseRotation.getRotation2d();
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in radians.
     */
    public double getRotation() {
        return poseRotation.getRotation();
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in degrees.
     */
    public double getRotationDEG() {
        return poseRotation.getRotationDEG();
    }


    // Set Values

    /** Updates this Rotation2ds rotation.
    * @param rotation value in radians.
    */
    public void setRotation(double rotation) {
        poseRotation.setRotation(rotation);
    }
}
