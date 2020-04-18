package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Turret extends Subsystem {
        private static Turret mInstance;
        private final TalonSRX mMaster;
        private double mTurretTicks;
        private final double mHomeAngle = 180;
    
        private Turret() {
            mMaster = new TalonSRX(Constants.kTurretMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            mMaster.setInverted(false);
            mMaster.setSensorPhase(true);
            mMaster.setNeutralMode(NeutralMode.Brake);
            mMaster.configForwardSoftLimitThreshold((int)degreeToTicks(310 - mHomeAngle), 0); // to the right
            mMaster.configReverseSoftLimitThreshold((int)degreeToTicks(-50 - mHomeAngle), 0); // to the left 40 degrees of deadzone
            mMaster.configForwardSoftLimitEnable(true);
            mMaster.configReverseSoftLimitEnable(true);

            mMaster.config_kP(0, Constants.kTurretKp);
            mMaster.config_kI(0, Constants.kTurretKi);
            mMaster.config_kD(0, Constants.kTurretKd);
        }
    
        public synchronized static Turret getInstance() {
            if (mInstance == null) {
                mInstance = new Turret();
            }
            return mInstance;
        }

        public void setOpenLoop(double output) {
            mMaster.set(ControlMode.PercentOutput, output);
        }

        public void setSetpoint(double angle) {
            double safeDegree;
            if (angle > 310) {
                safeDegree = angle - 360;
            } else if (angle < -50) { 
                safeDegree = angle + 360;
            } else
                safeDegree = angle;
            mMaster.set(ControlMode.Position, degreeToTicks(safeDegree - mHomeAngle));
        }

        public void faceTheTarget() {
            setSetpoint(Drive.getInstance().getHeadingDegree() % 360 + 180); 
        }

        public double getCurrentDegree() {
            return ticksToDegree(mTurretTicks) + mHomeAngle;
        }

        public double degreeToTicks(double degree) {
            return degree / 360 * 4096 * 12; // gear ratio is 1:12
        }

        public double ticksToDegree(double ticks) {
            return ticks / 4096 / 12 * 360;
        }

        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("Turret ticks", mTurretTicks);
            SmartDashboard.putNumber("Turret angle", ticksToDegree(mTurretTicks) + mHomeAngle);
        }
    
        @Override
        public void stop() {
            setOpenLoop(0);
        }
    
        @Override
        public void zeroSensors() {
            mMaster.setSelectedSensorPosition(0);
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
            mTurretTicks = mMaster.getSelectedSensorPosition(0);
        }
    
        @Override
        public void writePeriodicOutputs() {
        }
    
        @Override
        public boolean checkSystem() {
            return true;
        }
}
    