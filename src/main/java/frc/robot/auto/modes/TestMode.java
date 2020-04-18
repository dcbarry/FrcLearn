package frc.robot.auto.modes;

import frc.lib.util.DriveSignal;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;

public class TestMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mYolo1;

    public TestMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mYolo1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().test.get(mStartedLeft), true);
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two hatch backside rocket auto
        System.out.println("Running Yolo Program");
        runAction(new WaitAction(8));
        runAction(mYolo1);
    }
}
