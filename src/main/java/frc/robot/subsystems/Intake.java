package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Intake extends Subsystem {
        private static Intake mInstance;
        //private final VictorSPX mMaster;
        private final TalonSRX mMaster;
        private final Solenoid mSolenoid;

        private Intake() {
            //mMaster = new VictorSPX(Constants.kIntakeMasterId);
            mMaster = new TalonSRX(Constants.kIntakeMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(true);
            //mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            //mMaster.enableVoltageCompensation(true);

            mSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
        }
    
        public synchronized static Intake getInstance() {
            if (mInstance == null) {
                mInstance = new Intake();
            }
            return mInstance;
        }
    
        @Override
        public void outputTelemetry() {
        }
    
        @Override
        public void stop() {
            setPower(0);
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
                }
    
                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            };
            enabledLooper.register(loop);
        }
    
        public synchronized void setPower(double power) {
            mMaster.set(ControlMode.PercentOutput, power);
        }

        public synchronized void intake() {
            extend();
            setPower(1.0);
        }

        public synchronized void outtake() {
            retract();
            setPower(-0.0);
        }

        public void extend() {
            mSolenoid.set(true);
        }

        public void retract() {
            mSolenoid.set(false);
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
    