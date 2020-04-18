package frc.robot.auto.modes;

import java.util.Arrays;

import frc.lib.util.DriveSignal;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetIntake;
import frc.robot.auto.actions.SetShooter;
import frc.robot.auto.actions.SetTurret;
import frc.robot.auto.actions.SpeedUpShooter;
import frc.robot.auto.actions.StopIntake;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;

public class GreedyOppoTrench extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mFirstPath, mSecondTrussPath, mTrussPath;

    public GreedyOppoTrench(boolean robotStartedOnLeft) {
        mFirstPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().startToFifthBallOppoTrench.get(false), true);
        mSecondTrussPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().fifthBallOppoTrenchToTrussGoal.get(false));
        mTrussPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearGoalToThirdBallMiddleTruss.get(false));
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ParallelAction(
                Arrays.asList(
                        mFirstPath,
                        new SeriesAction(
                                Arrays.asList(
                                    new SetTurret(140),
                                    new SpeedUpShooter(0.55),  
                                    new WaitAction(0.3),
                                    new SetIntake(true)
                                )
                        )
                )
        ));

        // middle truss route
        runAction(new SeriesAction(
                Arrays.asList(
                        mSecondTrussPath,
                        new StopIntake(),
                        //new SetIntake(false),
                        new SetShooter(4.7, 5) 
                )
        ));
        /*runAction(new ParallelAction(
                Arrays.asList(
                        new SetShooter(4.7, 5),
                        new SeriesAction(
                                Arrays.asList(
                                    new WaitAction(0.5),
                                    new SetIntake(true),
                                    new WaitAction(0.5),
                                    new SetIntake(false)
                                )
                        )
                )
        ));*/

        runAction(new ParallelAction(
                Arrays.asList(
                        mTrussPath,
                        new SeriesAction(
                                Arrays.asList(
                                    new SetTurret(-215),
                                    new SpeedUpShooter(1.0),  
                                    new WaitAction(0.2),
                                    new SetIntake(true)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new SetShooter(5, 3),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.4),
                                        new SetIntake(false)
                                )
                                
                        )
                )
        ));
    }
}
