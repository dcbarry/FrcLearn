package frc.robot.auto.actions;

import frc.robot.subsystems.Shooter;

public class SpeedUpShooter implements Action {
    private static final Shooter mShooter = Shooter.getInstance();
    private double mShootingPower;

    public SpeedUpShooter(double power) {
        mShootingPower = power;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
        mShooter.setOpenLoop(mShootingPower);;
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }
}

