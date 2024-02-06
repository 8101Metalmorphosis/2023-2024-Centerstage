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


}
