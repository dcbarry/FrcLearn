package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class SetShooter implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Hopper mHopper = Hopper.getInstance();
    private double mStartTime, mTimeout;
    private int mStopOnBallCount, mInitialBallCount;

    public SetShooter(double timeout) {
        this(timeout, 10); // never possible
    }

    public SetShooter(double timeout, int ballCount) {
        mTimeout = timeout;
        mStopOnBallCount = ballCount;
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - mStartTime) > mTimeout) || ((mHopper.getBallCount() - mInitialBallCount) == mStopOnBallCount);
    }

    @Override
    public void update() {
        mSuperstructure.autoShoot();
        
    }

    @Override
    public void done() {
        mShooter.stop();
        mHopper.stop();
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.startAiming(true);
        mSuperstructure.forceAim();
        mInitialBallCount = mHopper.getBallCount();
        //Intake.getInstance().setPower(0.1);
        Drive.getInstance().setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
    }
}

