package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Hopper extends Subsystem {
        private static Hopper mInstance;
        private final VictorSPX mMaster, mSlave;
        //private final TalonSRX mMaster, mSlave;
        private final AnalogInput mDistanceSensor;
        private final TalonSRX mVerticalMaster;
        private int mBallCounter = 0;
        private boolean mPrevBallStatus = false;
        private boolean mBallStatus;
    
        private Hopper() {
            mMaster = new VictorSPX(Constants.kHopperMasterId);
            //mMaster = new TalonSRX(Constants.kHopperMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(false);
            mSlave = new VictorSPX(Constants.kHopperSlaveId);
            //mSlave = new TalonSRX(Constants.kHopperSlaveId);
            mSlave.setInverted(true);
            mSlave.follow(mMaster);
            mVerticalMaster = new TalonSRX(Constants.kHopperVerticalMasterId);

            mDistanceSensor = new AnalogInput(Constants.kHopperDistanceSensor);
        }
    
        public synchronized static Hopper getInstance() {
            if (mInstance == null) {
                mInstance = new Hopper();
            }
            return mInstance;
        }
    
        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("hopper sensor value", mDistanceSensor.getValue());
            SmartDashboard.putNumber("hopper ball count", getBallCount());
        }
    
        @Override
        public void stop() {
            setEntrancePower(0);
            setVerticalFeederPower(0);
        }
    
        @Override
        public void zeroSensors() {
        }
    
        @Override
        public void registerEnabledLoops(ILooper enabledLooper) {
            Loop loop = new Loop() {
    
                @Override
                public void onStart(double timestamp) {
                }
    
                @Override
                public void onLoop(double timestamp) {  
                    if (mDistanceSensor.getValue() > 1500)
                        mBallStatus = true;
                    else if (mDistanceSensor.getValue() < 900)
                        mBallStatus = false;

                    if (mPrevBallStatus != mBallStatus) {
                        if (mPrevBallStatus == true)
                            mBallCounter++;
                        mPrevBallStatus = mBallStatus;
                    }
                }
    
                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            };
            enabledLooper.register(loop);
        }

        public int getSensorValue() {
            return mDistanceSensor.getValue();
        }

        public int getBallCount() {
            return mBallCounter;
        }
    
        public void setEntrancePower(double power) { //synchronized
            mMaster.set(ControlMode.PercentOutput, power);
        }

        public void setVerticalFeederPower(double power) {
            mVerticalMaster.set(ControlMode.PercentOutput, power);
        }

        public void inflow() {
            double mX = Superstructure.getInstance().x;
            if (mX > 22)
                setEntrancePower(0.32);
            else if (mX > 19)
                setEntrancePower(0.35);
            else
                setEntrancePower(0.4);
        }

        public void outflow() {
            setEntrancePower(-0.4);
        }

        public void elevate() {
            setVerticalFeederPower(1.0);
        }

        public void descend() {
            setVerticalFeederPower(-1.0);
        }
    
        @Override
        public void readPeriodicInputs() {
        }
    
        @Override
        public void writePeriodicOutputs() {
        }
    
        @Override
        public boolean checkSystem() {
            return true;
        }
}
    