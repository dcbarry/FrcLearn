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

public class EightBallOurTrench extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mFirstPath, mSecondPath, mThirdPath, mFourthPath;

    public EightBallOurTrench(boolean robotStartedOnLeft) {
        mFirstPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().startToInnerFifthBallOurTrench.get(false), true);
        mSecondPath = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().innerFifthBallOurTrenchToShooting.get(false));
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two hatch backside rocket auto
        runAction(new SeriesAction(
                Arrays.asList(
                        new SetTurret(-180),
                        new SetShooter(3.0, 2) // minus 1 because of super stack
                )
        ));
        runAction(new ParallelAction(
                Arrays.asList(
                        mFirstPath,
                        new SeriesAction(
                                Arrays.asList(
                                    new SpeedUpShooter(0.45),
                                    new SetTurret(-135),
                                    new WaitAction(0.35),
                                    new SetIntake(true)
                                )
                        )
                )
        ));

        runAction(new SeriesAction(
                Arrays.asList(
                        mSecondPath,
                       new SetIntake(false),
                       new SetShooter(4.5, 4)
                )
        ));
    }
}
