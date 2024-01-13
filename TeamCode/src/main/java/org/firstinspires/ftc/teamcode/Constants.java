package org.firstinspires.ftc.teamcode;

public class Constants {

    public static class OtherConstants {
        public static float joystickThreshold = .1f;
        public static float triggerThreshhold = .2f;
    }

    public static class LifterConstants {
        public static int maxTicksPerLoop = 400;
        public static float lifterSpeed = .3f;

        public static int lifterZeroOffset = 10;
        public static int lifterAllowedThreshold = 50;


        public static float liftArmReset = .1f;
        public static float liftArmIdle = .3f;
        public static float liftArmIntake = .45f;
        public static float liftArmTop = .6f;
    }

    public static class ExtendConstants {
        public static int maxTicksPerLoop = 400;
        public static float extendSpeed = .3f;

        public static int extendZeroOffset = 10;
        public static int extendAllowedThreshold = 50;

        public static float intakeExtendArm = .1f;
        public static float resetExtendArm = 1f;

        public static float intakeSpeed = 1f;
    }

    public static class IntakeConstants {
        public static float doorClose = .175f;
        public static float doorOpen = .55f;
    }

    public static class ClawConstants {
        public static float wristTransfer = 0.165f;
        public static float wristDrop = .75f;

        public static float clawOpen = .25f;
        public static float clawClose = .6f;


    }

    public static class TimerConstants {
        // ALL TIMERS ARE PER .5 ROTATION

        // right now, all are guessed

        // Lifter
        public static int liftArmTimeMS = 400;
        public static int wristTimeMS = 200;
        public static int clawTimeMS = 100;

        // Extend
        public static int extendArmTimeMS = 400;
        public static int doorTimeMS = 250;

    }
}
