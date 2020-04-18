package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;

public class StopIntake implements Action {
    private static final Intake mIntake = Intake.getInstance();

    public StopIntake() {
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
        mIntake.stop();
    }
}

