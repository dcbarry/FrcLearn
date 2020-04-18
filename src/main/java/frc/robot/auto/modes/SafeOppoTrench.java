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

public class SafeOppoTrench extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mFirstPath, mSecondTrussPath, mTrussPath, mTrussSecondPath;

    public SafeOppoTrench(boolean robotStartedOnLeft) {
        mFirstPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().startToThirdBallOppoTrench.get(false), true);
        mSecondTrussPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().thirdBallOppoTrenchToTrussGoal.get(false));
        mTrussPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearGoalToThirdBallMiddleTruss.get(false));
        mTrussSecondPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().thirdBallMiddleTrussToShooting.get(false));
        //mFourthPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().thirdBallOurTrenchToShooting.get(false));
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
                                    new SetTurret(-220),
                                    new SpeedUpShooter(0.45),  
                                    new WaitAction(0.25),
                                    new SetIntake(true)
                                )
                        )
                )
        ));

        // middle truss route
        runAction(new SeriesAction(
                Arrays.asList(
                        new StopIntake(),
                        mSecondTrussPath,
                        new SetIntake(false)
                        
                )
        ));
        runAction(new ParallelAction(
                Arrays.asList(
                        new SetShooter(4.5, 5),
                        new SeriesAction(
                                Arrays.asList(
                                    new WaitAction(0.5),
                                    new SetIntake(true),
                                    new WaitAction(0.3),
                                    new SetIntake(false)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        mTrussPath,
                        new SeriesAction(
                                Arrays.asList(
                                    new SetTurret(-220),
                                    new SpeedUpShooter(0.6),  
                                    new WaitAction(0.15),
                                    new SetIntake(true)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        //new SetIntake(false),
                        //mTrussSecondPath,
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
