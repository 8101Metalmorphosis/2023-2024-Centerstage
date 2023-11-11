package org.firstinspires.ftc.teamcode.FTCutil.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double Kp;
    double Ki;
    double Kd;


    int target;
    int current;

    double integralSum = 0;
    double lastError = 0;


    ElapsedTime timer = new ElapsedTime();

    public PIDController(double p, double i, double d, int initialPos) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;

        this.target = initialPos;
    }

    public void setTargetPos(int value) {
        this.target = value;
    }

    public double update(int error) {


        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        lastError = error;

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public double getCurrentPosition() {
        return current;
    }

    public void setPID(double p, double i, double d) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }
}
