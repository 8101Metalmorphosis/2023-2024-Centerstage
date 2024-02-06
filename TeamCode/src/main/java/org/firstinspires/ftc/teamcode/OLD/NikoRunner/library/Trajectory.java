package org.firstinspires.ftc.teamcode.OLD.NikoRunner.library;

import java.util.ArrayList;

public class Trajectory {

    ArrayList<SplineValues2d> paths;

    static ArrayList<Spline2d> splineList = new ArrayList<>();
    
   public ArrayList<Spline2d> getPoints() {
        return splineList;
   }

    public Trajectory(TrajectoryBuilder builder) {
        this.paths = builder.splinePath;
    }

    /** 
     * @return returns a trajectories end pose
     */
    public Pose2d end() {
        return splineList.get(splineList.size()).getEndPose();
    }


    public static class TrajectoryBuilder {
        ArrayList<SplineValues2d> splinePath = new ArrayList<>();
        ArrayList<SplineValues2d> linePath = new ArrayList<>();
        Pose2d startPose;

        public TrajectoryBuilder(Pose2d startPose) {
            this.startPose = startPose;
        }

        public Trajectory build() {

            Boolean lastContinuity = false;
            

            for (int i = 0; i < splinePath.size(); i++) {

                Pose2d initialPose;
                Rotation2d initialTangent;

                // If first spline in trajectory, reset initial pose and tangent, otherwise, set to last spline end pose.
                // Also reset continuity to beginning spline.
                if(i == 0) {
                    initialPose = startPose;
                    initialTangent = null;

                    lastContinuity = splinePath.get(i).type.equals("splineToProfiledHeading") || 
                    splinePath.get(i).type.equals("splineLineToProfiledHeading");
                } else {
                    initialPose = splinePath.get(i - 1).getPose();
                    initialTangent = new Rotation2d(Math.signum(splinePath.get(i - 1).getRotation()) * -(splinePath.get(i - 1).getRotation() + Math.PI));
                }


                boolean continuous = splinePath.get(i).type.equals("splineToProfiledHeading") || 
                splinePath.get(i).type.equals("splineLineToProfiledHeading");
                
                lastContinuity = continuous;


                if (continuous) {

                    SplineGenerator spline = new SplineGenerator(
                        initialPose, 
                        initialTangent, 
                        splinePath.get(i).endPose, 
                        splinePath.get(i).getRotation2d(),
                        splinePath.get(i).getEndTanDistance(),
                        splinePath.get(i).getType()
                    );

                    splineList.add(spline.getSpline());
                } else {

                    LineGenerator line = new LineGenerator(
                        initialPose, 
                        splinePath.get(i).endPose,
                        splinePath.get(i).getType()
                    );

                    splineList.add(line.getAsSpline());
                }
            }

            return new Trajectory(this);
        }

        // Splines
        public TrajectoryBuilder splineToProfiledHeading(Pose2d endPose, Rotation2d endTangent, double endTangentDistance) {
            this.splinePath.add(new SplineValues2d(endPose, endTangent, endTangentDistance, "splineToProfiledHeading"));
            return this;
        }

        public TrajectoryBuilder splineToLinearHeading(Pose2d endPose, Rotation2d endTangent, double endTangentDistance) {
            this.splinePath.add(new SplineValues2d(endPose, endTangent, endTangentDistance, "splineToLinearHeading"));
            return this;
        }

        // make heading uselessl
        public TrajectoryBuilder splineToConstantHeading(Vector2d endVector, Rotation2d endTangent, double endTangentDistance) {
            this.splinePath.add(new SplineValues2d(new Pose2d(endVector), endTangent, endTangentDistance, "splineToConstantHeading"));
            return this;
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

        // make heading useless
        public TrajectoryBuilder splineTo(Pose2d endPose, Rotation2d endTangent, double endTangsntDistance) {
            this.splinePath.add(new SplineValues2d(endPose, endTangent, endTangsntDistance, "splineTo"));
            return this;
        }

        // SplineLines
        public TrajectoryBuilder splineLineToProfiledHeading(Pose2d endPose, Rotation2d endTangent, double endTangentDistance) {
            this.splinePath.add(new SplineValues2d(endPose, endTangent, endTangentDistance, "splineLineToProfiledHeading"));
            return this;
        }

        // Lines
        public TrajectoryBuilder lineToProfiledHeading(Pose2d endPose) {
            this.splinePath.add(new SplineValues2d(endPose, null, 0, "lineToProfiledHeading"));
            return this;
        }
    }
}