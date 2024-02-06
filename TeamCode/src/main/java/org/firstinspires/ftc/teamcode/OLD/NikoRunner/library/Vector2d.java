package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

public class Vector2d {
    private double x;
    private double y;

    /** Creates a new Vector2d class.
    * @param x the x value for this Vector2d.
    * @param y the y value for this Vector2d.
    */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Returned values

    /**
     * @return returns as a vector.
     */
    public Vector2d getVector() {
        return this;
    }

    /**
     * @return returns this vectors x value.
     */
    public double getX() {
        return x;
    }

    /**
     * @return returns this vectors y value.
     */
    public double getY() {
        return y;
    }


    // Set values

    /** Updates this Vector2ds x and y coordinates.
    * @param x value.
    * @param y value.
    */
    public void setVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @param x the new x value of this Vector2d.
     */
    public void setX(double x) {
        this.x = x;
    }
    
    /**
     * @param y the new y value of this Vector2d.
     */
    public void setY(double y) {
        this.y = y;
    }
}


