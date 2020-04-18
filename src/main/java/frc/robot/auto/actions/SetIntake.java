package frc.robot.auto.actions;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

public class SetIntake implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private boolean whetherIntake;

    public SetIntake(boolean intake) {
        whetherIntake = intake;
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
        Hopper.getInstance().stop();
    }

    @Override
    public void start() {
        if (whetherIntake) {
            mSuperstructure.autoIntake();
        }
        else {
            mIntake.stop();
            mIntake.retract();
            Hopper.getInstance().stop();
        } 

    }
}

