package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Limelight extends Subsystem {
    private static Limelight mInstance;
    private NetworkTable mMainLimelight = NetworkTableInstance.getDefault().getTable("limelight-main");

    private double mLimelightTx, mLimelightTy;
    private boolean mLightStatus = true;

    public synchronized static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
    }

    
    @Override
    public void registerEnabledLoops(ILooper looper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }

    public void setLEDLight(boolean on) {
        if (mLightStatus != on) {
            if (on)
                mMainLimelight.getEntry("ledMode").setDouble(3); //1 = off, 3 = on
            else
                mMainLimelight.getEntry("ledMode").setDouble(1); 
            mLightStatus = on;
        }
    }

    public double getTargetX() {
        return mLimelightTx;
    }

    public double getTargetY() {
        return mLimelightTy;
    }

    public double getEstimatedDistance() {
        return (98.0-7-24.0) / Math.tan(Math.toRadians(mLimelightTy+32.0));
    }

    public boolean getTargetExists() {
        return mMainLimelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return mMainLimelight.getEntry("ta").getDouble(0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putNumber("limelight tx", mLimelightTx);
        //SmartDashboard.putNumber("limelight ty", mLimelightTy);
        SmartDashboard.putNumber("limelight distance", getEstimatedDistance());
    }

    @Override
    public void readPeriodicInputs() {
        mLimelightTx = mMainLimelight.getEntry("tx").getDouble(0);
        mLimelightTy = mMainLimelight.getEntry("ty").getDouble(0);
    }
    
    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void stop() {
    }
}