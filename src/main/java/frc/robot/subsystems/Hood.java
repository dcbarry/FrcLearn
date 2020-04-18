package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Hood extends Subsystem {
        private static Hood mInstance;
        private final TalonSRX mMaster;
        private final int homeAngle = 50;
        private int ticks;
    
        private Hood() {
            mMaster = new TalonSRX(Constants.kHoodMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); // maybe absolute encoder
            mMaster.setInverted(false);
            mMaster.setSensorPhase(false);
            mMaster.setNeutralMode(NeutralMode.Brake);
            mMaster.configForwardSoftLimitThreshold((int)degreeToTicks(homeAngle - 16.0), 0); // flattest degree
            mMaster.configReverseSoftLimitThreshold((int)degreeToTicks(homeAngle - 50), 0); // deepest degree
            mMaster.configForwardSoftLimitEnable(true);
            mMaster.configReverseSoftLimitEnable(true);

            mMaster.config_kP(0, Constants.kHoodKp);
            mMaster.config_kI(0, Constants.kHoodKi);
            mMaster.config_kD(0, Constants.kHoodKd);
        }
    
        public synchronized static Hood getInstance() {
            if (mInstance == null) {
                mInstance = new Hood();
            }
            return mInstance;
        }

        public void setOpenLoop(double output) {
            mMaster.set(ControlMode.PercentOutput, output);
        }

        public void setSetpoint(double angleFromGround) {
            double safeAngle;
            if (angleFromGround < 16.0)
                safeAngle = 16.0;
            else
                safeAngle =  angleFromGround;
            mMaster.set(ControlMode.Position, degreeToTicks(homeAngle - safeAngle));
        }

        private double degreeToTicks(double degree) {
            return degree / 360 * 4096 * (297 / 14); // suppose the ratio is 1:70 so
        }

        private double ticksToDegree(int ticks) {
            return (double) ticks / 4096 / 297 * 14 * 360;
        }

        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("Hood angle", homeAngle - ticksToDegree(ticks));
            SmartDashboard.putNumber("Hood ticks", ticks);
        }
    
        @Override
        public void stop() {
            setOpenLoop(0);
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
            ticks = mMaster.getSelectedSensorPosition(0);
        }
    
        @Override
        public void writePeriodicOutputs() {
        }
    
        @Override
        public boolean checkSystem() {
            return true;
        }
}
    