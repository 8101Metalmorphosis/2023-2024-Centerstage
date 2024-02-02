package org.firstinspires.ftc.teamcode;

public class Constants {

    public static class OtherConstants {
        public static float joystickThreshold = .1f;
        public static float triggerThreshhold = .2f;
    }

    public static class LifterConstants {
        public static int maxTicksPerLoop = 400;
        public static float lifterSpeed = .7f;
        public static float lifterZeroSpeed = -.2f;

        public static int lifterMaxHeight = 2000;
        public static int lifterMinHeight = 30;


        public static int lifterAllowedThreshold = 50;


        public static float liftArmIdle = .4f;
        public static float liftArmTransfer = .128f;
        public static float liftArmTop = .85f;
    }

    public static class ExtendConstants {
        public static int maxTicksPerLoop = 800;
        public static float extendSpeed = .75f;
        public static float extendZeroSpeed = -.2f;

        public static int extendMinHeight = 25;
        public static int extendMaxHeight = 3800;
        public static int extendAllowedThreshold = 50;

        public static float intakeExtendArm = .2f;
        public static float resetExtendArm = .9f;
    }

    public static class IntakeConstants {
        public static float doorClose = .1f;
        public static float doorOpen = .7f;

        public static float intakeSpeed = 1f;
    }

    public static class ClawConstants {
        public static float wristPitchTransfer = 0.125f;
        public static float wristPitchDrop = .75f;

        public static float wristRollVertical = .5f;
        public static float wristRollLeft = .85f;
        public static float wristRollRight = .15f;

        public static float clawFullOpen = .1f;
        public static float clawOpen = .25f;
        public static float clawClose = .7f;
    }

    public static class TimerConstants {
        // ALL TIMERS ARE PER .5 ROTATION

        // Lifter
        public static int liftArmTimeMS = 250;
        public static int wristTimeMS = 500;
        public static int clawTimeMS = 2500;

        // Extend
        public static int extendArmTimeMS = 1000;
        public static int doorTimeMS = 500;

    }
}
