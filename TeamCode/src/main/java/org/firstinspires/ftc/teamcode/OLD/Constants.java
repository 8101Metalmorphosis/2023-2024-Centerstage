package org.firstinspires.ftc.teamcode.OLD;

public class Constants {
    public static class Chassis {
        public static final double rotationConstraintsDEG[] = {5, -1, 5};
        public static final double[] rotationConstraintsRAD =
                                {Math.toRadians(rotationConstraintsDEG[0]),
                                    Math.toRadians(rotationConstraintsDEG[1]),
                                    Math.toRadians(rotationConstraintsDEG[2])};

        public static final double ticksPerDegree = 10;
    }

    public class Lifter {

        public static final int maxLength = 300;
        public static final int lowLength = 0;

        public static final int maxAccel = 1200;
        public static final int maxVel = 800;

        public static final int initialPos = 0;


        public static final int maxTickLength = 250;

        // PHYSICAL
        public static final int ticksPerInch = 50;

        public static final int angleDEG = 50;
    }

    public class Claw {

        public static final double open = 0.55;
        public static final double close = .25;

    }

    public class Intake {

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
