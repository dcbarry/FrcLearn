package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Climber extends Subsystem {
        private static Climber mInstance;
        private final TalonSRX mMaster, mSlave;
        private final Solenoid mSolenoid;
    
        private Climber() {
            //mMaster = new VictorSPX(Constants.kIntakeMasterId);
            mMaster = new TalonSRX(Constants.kClimberMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(false);
            mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            mMaster.enableVoltageCompensation(true);
            mMaster.setNeutralMode(NeutralMode.Brake);

            mSlave = new TalonSRX(Constants.kClimberSlaveId);
            mSlave.setInverted(true);
            mSlave.follow(mMaster);

            mSolenoid = new Solenoid(Constants.kClimberSolenoidId);
        }
    
        public synchronized static Climber getInstance() {
            if (mInstance == null) {
                mInstance = new Climber();
            }
            return mInstance;
        }

        public synchronized void setPower(double power) {
            mMaster.set(ControlMode.PercentOutput, power);
        }

        public synchronized void ascend() {
            setPower(1.0);
        }

        public synchronized void descend() {
            setPower(-1.0);
        }

        public void foldout() {
            mSolenoid.set(true);
            Intake.getInstance().retract();
        }

        public void retract() {
            mSolenoid.set(false);
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
    