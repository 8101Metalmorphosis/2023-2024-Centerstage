package org.firstinspires.ftc.teamcode.FTCutil.PID;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class ProfiledPIDController {

    PIDController pid;

    double Kp;
    double Ki;
    double Kd;

    double pos;

    double maxAccel;
    double maxVel;

    int target;
    int current;



    ElapsedTime timer = new ElapsedTime();

    public ProfiledPIDController(double p, double i, double d, double maxAccel, double maxVel, int initialPosition) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;

        this.maxAccel = maxAccel;
        this.maxVel = maxVel;

        pid = new PIDController(p, i, d, initialPosition);
    }

    private double betterCalculate(int currentPosition, int targetPosition) {

        double time = timer.seconds();


        double distance = (targetPosition - currentPosition);


        double acceleration_dt = maxVel / maxAccel;

        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);

        if(acceleration_distance > halfway_distance * Math.signum(distance)) {
            acceleration_dt = Math.sqrt(halfway_distance * Math.signum(distance) /  (0.5 * maxAccel));
        }

        acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);


        double calcMaxVel = maxAccel * acceleration_dt;

        double deacceleration_dt = acceleration_dt;


        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / calcMaxVel;
        double deacceleration_time = acceleration_dt + cruise_dt;

//        System.out.println("time: " + time);
//        System.out.println("distance: " + distance);
//        System.out.println("accel dt: " + acceleration_dt);
//        System.out.println("cruise distance: " + cruise_distance);
//        System.out.println("cruise dt: " + cruise_dt);


        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;

        if (time > entire_dt) {
            System.out.println("FINISHED");
            return distance;
        }

        if (time < acceleration_dt) {
            System.out.println("ACCEL");
            return 0.5 * maxAccel * Math.pow(time, 2);
        } else if (time < deacceleration_time) {
            System.out.println("CRUISING");
            acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = time - acceleration_dt;

            return acceleration_distance + maxVel + cruise_current_dt;
        } else {
            System.out.println("DECCEL");
            acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);
            cruise_distance = maxVel * cruise_dt;
            deacceleration_time = time - deacceleration_time;

            return acceleration_distance + cruise_distance + maxVel * deacceleration_time - 0.5 * maxAccel * Math.pow(deacceleration_time, 2);
        }
    }

    public double update(int currentPosition, int targetPosition) {
        this.current = currentPosition;
        this.target = targetPosition;

        System.out.println("Arm Distance (from target): " + (targetPosition - currentPosition));
        pos = betterCalculate(currentPosition, targetPosition);

        double calculatedValue = pid.update((int) pos);

        return calculatedValue;
    }

    public double getInstantTarget(int currentPosition, int targetPosition) {
        return betterCalculate(currentPosition, targetPosition);
    }

    public double getCurrentPos() {
        return pos;
    }


    public void resetTimer() {
        timer.reset();
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
