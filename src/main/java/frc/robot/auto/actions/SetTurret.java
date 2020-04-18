package frc.robot.auto.actions;

import frc.robot.subsystems.Turret;

public class SetTurret implements Action {
    private static final Turret mTurret = Turret.getInstance();
    private double turretAngle;

    public SetTurret(double degree) {
        turretAngle = degree;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mTurret.setSetpoint(turretAngle);
    }
}

