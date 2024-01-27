package org.firstinspires.ftc.teamcode;

public class Constants {

    public static class OtherConstants {
        public static float joystickThreshold = .1f;
        public static float triggerThreshhold = .2f;
    }

    public static class LifterConstants {
        public static int maxTicksPerLoop = 400;
        public static float lifterSpeed = .7f;
        public static float lifterZeroSpeed = -.3f;

        public static int lifterMaxHeight = 2000;
        public static int lifterMinHeight = 100;


        public static int lifterAllowedThreshold = 50;


        public static float liftArmReset = .1f;
        public static float liftArmIdle = .3f;
        public static float liftArmIntake = .45f;
        public static float liftArmTop = .65f;
    }

    public static class ExtendConstants {
        public static int maxTicksPerLoop = 400;
        public static float extendSpeed = .3f;

        public static int extendZeroOffset = 10;
        public static int extendAllowedThreshold = 50;

        public static float intakeExtendArm = .1f;
        public static float resetExtendArm = .82f;
    }

    public static class IntakeConstants {
        public static float doorClose = .1f;
        public static float doorOpen = .7f;

        public static float intakeSpeed = 1f;
    }

    public static class ClawConstants {
        public static float wristPitchTransfer = 0.165f;
        public static float wristPitchDrop = .75f;

        public static float wristRollVertical = .5f;
        public static float wristRollLeft = .85f;
        public static float wristRollRight = .15f;

        public static float clawOpen = .21f;
        public static float clawClose = .7f;
    }

    public static class TimerConstants {
        // ALL TIMERS ARE PER .5 ROTATION

        // Lifter
        public static int liftArmTimeMS = 1000;
        public static int wristTimeMS = 1000;
        public static int clawTimeMS = 1000;

        // Extend
        public static int extendArmTimeMS = 1000;
        public static int doorTimeMS = 1000;

    }
}
