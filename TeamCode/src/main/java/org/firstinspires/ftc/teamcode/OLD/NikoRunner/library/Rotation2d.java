package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

public class Rotation2d {

    double rotation;

    /** Creates a new Rotation2d class.
     * @param rotation the rotation in radians for this Rotation2d.
     */
    public Rotation2d(double rotation) {
        this.rotation = rotation;
    }


    // Returned values

    /**
     * @return returns this Rotation2d as a Rotation2d.
     */
    public Rotation2d getRotation2d() {
        return this;
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in radians.
     */
    public double getRotation() {
        return rotation;
    }

    /**
     * @return returns this Rotation2ds rotation as a rotation in degrees.
     */
    public double getRotationDEG() {
        return Math.toDegrees(rotation);
    }


    // Set Values

    /** Updates this Rotation2ds rotation.
    * @param rotation value in radians.
    */
    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

}
