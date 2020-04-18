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
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;

public class FiveBallOppoTrench extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mFirstPath, mSecondPath;

    public FiveBallOppoTrench(boolean robotStartedOnLeft) {
        mFirstPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().startToFifthBallOppoTrench.get(false), true);
        mSecondPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().fifthBallOppoTrenchToTrussGoal.get(false));
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two hatch backside rocket auto
        runAction(new ParallelAction(
                Arrays.asList(
                        mFirstPath,
                        new SeriesAction(
                                Arrays.asList(
                                    new SetTurret(90),
                                    new WaitAction(0.25),
                                    new SetIntake(true)
                            )
                    )
                )
        ));

        // Start driving path3; picking the second hatch
        runAction(new SeriesAction(
                Arrays.asList(
                        new SetIntake(false),
                        new SpeedUpShooter(0.4),  
                        mSecondPath,
                        new SetShooter(4)
                )
        ));
    }
}
