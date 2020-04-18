package frc.robot.paths;

import frc.robot.planners.DriveMotionPlanner;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 150.0; //115
    private static final double kMaxAccel = 160.0; //130
    private static final double kMaxCentripetalAccel = 110.0; // maybe higher
    private static final double kMaxVoltage = 9.0;

    private static final double kIntakeVelocity = 90.0;
    private static final double kMinVelocity = 66.0; // less the better
    private static final double kMinAccel = 80;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    // 10 ball oppo trench
    public static final Pose2d kOppoTrenchStartPose = new Pose2d(500.0, 134.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kOppoTrenchThirdBallPose = new Pose2d(340.0, 135.0, Rotation2d.fromDegrees(190.0));
    public static final Pose2d kOppoTrenchFifthBallPose = new Pose2d(257.0, 134.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kNearGoalPose = new Pose2d(465.0, -60.0, Rotation2d.fromDegrees(90.0)); // far pose
    public static final Pose2d kAdjustedNearGoalPose = new Pose2d(465.0, -60.0, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d kOurTrenchInnerFifthBallPose = new Pose2d(264.0, -128.0, Rotation2d.fromDegrees(170.0));
    public static final Pose2d kOurTrenchOuterFifthBallPose = new Pose2d(266.0, -142.0, Rotation2d.fromDegrees(190.0));
    public static final Pose2d kOurTrenchThirdBallPose = new Pose2d(340.0, -133.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kTrussGoalPose = new Pose2d(482.0, -30.0, Rotation2d.fromDegrees(115.0));
    public static final Pose2d kTrussThirdBallPose = new Pose2d(420.0, -5.0, Rotation2d.fromDegrees(200.0));
    

    // 8 ball our trench
    public static final Pose2d kOurTrenchStartPose = new Pose2d(500.0, -65.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kOurTrenchSecondBallPose = new Pose2d(375.0, -133.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kOurTrenchShootingPose = new Pose2d(490.0, -60.0, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d kOurTrenchBufferPose = new Pose2d(300.0, -133.0, Rotation2d.fromDegrees(180.0));
    // our trench fifth ball pose repeated above 

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState <Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory test;
        public final MirroredTrajectory startToThirdBallOppoTrench;
        public final MirroredTrajectory thirdBallOppoTrenchToNearGoal;
        public final MirroredTrajectory thirdBallOppoTrenchToTrussGoal;
        public final MirroredTrajectory adjustedNearGoalToFifthBallOurTrench;
        public final MirroredTrajectory adjustedNearGoalToThirdBallOurTrench;
        public final MirroredTrajectory nearGoalToThirdBallMiddleTruss;
        public final MirroredTrajectory thirdBallMiddleTrussToShooting;
        public final MirroredTrajectory outerFifthBallOurTrenchToShooting;
        public final MirroredTrajectory innerFifthBallOurTrenchToShooting;

        public final MirroredTrajectory startToFifthBallOppoTrench;
        public final MirroredTrajectory fifthBallOppoTrenchToTrussGoal;

        public final MirroredTrajectory startToInnerFifthBallOurTrench;
        public final MirroredTrajectory innerFifthBallToBuffer;
        public final MirroredTrajectory bufferToOuterFifthBall;

        private TrajectorySet() {
            test = new MirroredTrajectory(test());

            startToThirdBallOppoTrench = new MirroredTrajectory(getStartToThirdBallOppoTrench());
            thirdBallOppoTrenchToNearGoal = new MirroredTrajectory(getThirdBallOppoTrenchToNearGoal());
            thirdBallOppoTrenchToTrussGoal = new MirroredTrajectory(getThirdBallOppoTrenchToTrussGoal());
            adjustedNearGoalToFifthBallOurTrench = new MirroredTrajectory(getAdjustedNearGoalToFifthBallOurTrench());
            adjustedNearGoalToThirdBallOurTrench = new MirroredTrajectory(getAdjustedNearGoalToThirdBallOurTrench());
            nearGoalToThirdBallMiddleTruss = new MirroredTrajectory(getNearGoalToThirdBallMiddleTruss());
            thirdBallMiddleTrussToShooting = new MirroredTrajectory(getThirdBallMiddleTrussToShooting());
            outerFifthBallOurTrenchToShooting = new MirroredTrajectory(getOuterFifthBallOurTrenchToShooting());
            innerFifthBallOurTrenchToShooting = new MirroredTrajectory(getInnerFifthBallOurTrenchToShooting());

            startToFifthBallOppoTrench = new MirroredTrajectory(getStartToFifthBallOppoTrench());
            fifthBallOppoTrenchToTrussGoal = new MirroredTrajectory(getFifthBallOppoTrenchToTrussGoal());

            startToInnerFifthBallOurTrench = new MirroredTrajectory(getStartToInnerFifthBallOurTrench());
            innerFifthBallToBuffer = new MirroredTrajectory(getInnerFifthBallToBuffer());
            bufferToOuterFifthBall = new MirroredTrajectory(getBufferToOuterFifthBall());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> test() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(220.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(230.0, -50.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    40, 40, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToThirdBallOppoTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOppoTrenchStartPose);
            waypoints.add(new Pose2d(new Translation2d(400.0, 138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kOppoTrenchThirdBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdBallOppoTrenchToNearGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOppoTrenchThirdBallPose);
            waypoints.add(kNearGoalPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdBallOppoTrenchToTrussGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOppoTrenchThirdBallPose);
            waypoints.add(kTrussGoalPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getAdjustedNearGoalToFifthBallOurTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kAdjustedNearGoalPose);
            waypoints.add(new Pose2d(new Translation2d(400.0, -133.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kOurTrenchInnerFifthBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getAdjustedNearGoalToThirdBallOurTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kAdjustedNearGoalPose);
            waypoints.add(kOurTrenchThirdBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getNearGoalToThirdBallMiddleTruss() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTrussGoalPose);
            //waypoints.add(new Pose2d(new Translation2d(450.0, -10.0), Rotation2d.fromDegrees(160.0)));
            waypoints.add(kTrussThirdBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdBallMiddleTrussToShooting() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTrussThirdBallPose);
            waypoints.add(kTrussGoalPose);
            //waypoints.add(new Pose2d(new Translation2d(450.0, -10.0), Rotation2d.fromDegrees(160.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getOuterFifthBallOurTrenchToShooting() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOurTrenchOuterFifthBallPose);
            waypoints.add(kOurTrenchShootingPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getInnerFifthBallOurTrenchToShooting() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOurTrenchInnerFifthBallPose);
            waypoints.add(new Pose2d(new Translation2d(340.0, -133.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kOurTrenchShootingPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToFifthBallOppoTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOppoTrenchStartPose);
            //waypoints.add(new Pose2d(new Translation2d(390.0, 138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kOppoTrenchFifthBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getFifthBallOppoTrenchToTrussGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOppoTrenchFifthBallPose);
            waypoints.add(new Pose2d(new Translation2d(340.0, 134.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kTrussGoalPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToInnerFifthBallOurTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOurTrenchStartPose);
            waypoints.add(new Pose2d(new Translation2d(430.0, -130.0), Rotation2d.fromDegrees(192.0)));
            waypoints.add(kOurTrenchInnerFifthBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getInnerFifthBallToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOurTrenchInnerFifthBallPose);
            waypoints.add(kOurTrenchBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToOuterFifthBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOurTrenchBufferPose);
            waypoints.add(kOurTrenchOuterFifthBallPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
