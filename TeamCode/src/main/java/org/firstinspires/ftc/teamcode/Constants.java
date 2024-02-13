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

        public static int lifterMaxHeight = 2400;
        public static int lifterMinHeight = 10;


        public static int lifterAllowedThreshold = 50;


        public static float liftArmIdle = .4f;
        public static float liftArmIntake = .14f;
        public static float liftArmTransfer = .128f;

        public static float liftArmTop = .85f;
        public static float liftArmTop2 = .95f;
    }

    public static class ExtendConstants {
        public static int maxTicksPerLoop = 400;
        public static float extendSpeed = 1f;
        public static float extendZeroSpeed = -.2f;

        public static int extendMinHeight = 5;
        public static int extendMaxHeight = 2000;
        public static int extendAllowedThreshold = 10;

        public static float intakeExtendArm = .1f;
        public static float resetExtendArm = .8f;


        // GUESSES
        public static float stackIntake5 = .19f;
        public static float stackIntake4 = .16f;
        public static float stackIntake3 = .15f;
        public static float stackIntake2 = .14f;

    }

    public static class IntakeConstants {
        public static float doorClose = .1f;
        public static float doorOpen = .7f;

        public static float intakeSpeed = 1f;
        public static float autoOuttakeSpeed = -.35f;
    }

    public static class ClawConstants {
        public static float wristPitchTransfer = 0.125f;
        public static float wristPitchDrop = .75f;
        public static float wristPitchDrop2nd = .80f;
        public static float wristPitchDrop2 = .68f;

        public static float wristRollVertical = .5f;
        public static float wristRollLeft = .85f;
        public static float wristRollLeftHalf = .65f;
        public static float wristRollRight = .15f;
        public static float wristRollRightHalf = .35f;

        public static float clawFullOpen = .1f;
        public static float clawOpen = .25f;
        public static float clawClose = .7f;
    }

    public static class TimerConstants {
        // ALL TIMERS ARE PER .5 ROTATION

        // Lifter
        public static int liftArmTimeMS = 100;
        public static int wristTimeMS = 100;
        public static int clawTimeMS = 2500;

        // Extend
        public static int extendArmTimeMS = 700;
        public static int doorTimeMS = 100;
    }

    public static class AprilTags {

        public static final int ID_BLUEALLIANCE_WALLSMALL = 9;
        public static final int ID_BLUEALLIANCE_LEFT = 1;
        public static final int ID_BLUEALLIANCE_CENTER = 2;
        public static final int ID_BLUEALLIANCE_RIGHT = 3;

        public static final int ID_REDALLIANCE_WALLSMALL = 8;
        public static final int ID_REDALLIANCE_LEFT = 4;
        public static final int ID_REDALLIANCE_CENTER = 5;
        public static final int ID_REDALLIANCE_RIGHT = 6;
    }
}
