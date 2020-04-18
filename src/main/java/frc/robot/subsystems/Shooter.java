package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Shooter extends Subsystem {
        private static Shooter mInstance;
        private final TalonSRX mMaster, mSlave;
        private boolean mOnTarget;
        private double shooterTicksPer100ms;
    
        private Shooter() {
            mMaster = new TalonSRX(Constants.kShooterMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            mMaster.setInverted(false);
            mMaster.setSensorPhase(false);
            mMaster.setNeutralMode(NeutralMode.Coast);
            mMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, 10); //10ms
            mMaster.configVelocityMeasurementWindow(1); //maybe lower 32
            mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            mMaster.enableVoltageCompensation(true);
            mMaster.configPeakOutputReverse(-0.1);
            mMaster.configPeakOutputForward(1.0);
            mMaster.configClosedLoopPeakOutput(0, 1);
            mMaster.enableCurrentLimit(false);
            mMaster.configOpenloopRamp(0.1);

            mMaster.config_kP(0, Constants.kShooterKp);
            mMaster.config_kI(0, Constants.kShooterKi);
            mMaster.config_kD(0, Constants.kShooterKd);
            mMaster.config_kF(0, Constants.kShooterKf);

            mSlave = new TalonSRX(Constants.kShooterSlaveId);
            mSlave.follow(mMaster);
            mSlave.setInverted(false);
        }
    
        public synchronized static Shooter getInstance() {
            if (mInstance == null) {
                mInstance = new Shooter();
            }
            return mInstance;
        }

        public void setOpenLoop(double output) {
            mMaster.set(ControlMode.PercentOutput, output);
        }

        public void setBangBang(double rpm) {
            if (rpm - ticksPer100msToRpm(shooterTicksPer100ms) > 5) { // desire speed - current speed
                setOpenLoop(1);
                mOnTarget = true;
            } else if (rpm - ticksPer100msToRpm(shooterTicksPer100ms) < -5) {
                setOpenLoop(0.017125*rpmToTicksPer100ms(rpm)/1023); //0.017125
                mOnTarget = false;
            } else {
                if (mOnTarget)
                    setOpenLoop(1);
                else
                    setOpenLoop(0.017125*rpmToTicksPer100ms(rpm)/1023); 
            }
        }

        public void setPFControl(double rpm) {
            mMaster.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm));
        }

        public void setSetpointRPM(double rpm) {
            mMaster.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm));
        }

        public void setSetpointVknot(double Vknot) {
            mMaster.set(ControlMode.Velocity, vknotToTicksPer100ms(Vknot));
        }

        public boolean hasReachedDesiredSpeed(double rpm) {
            return Math.abs(rpm - ticksPer100msToRpm(shooterTicksPer100ms)) < 20; 
        }

        public boolean whetherForceStop(double rpm) {
            return Math.abs(rpm - ticksPer100msToRpm(shooterTicksPer100ms)) > 80; 
        }

        public boolean onTarget() {
            return mOnTarget;
        }

        private double ticksPer100msToRpm(double angularVelocity) {
            return angularVelocity * 600. / 4096.;
        }

        private double rpmToTicksPer100ms(double rpm) { 
            return rpm * 4096 / 600;
        }

        public double vknotToTicksPer100ms(double Vknot) {
            return Vknot/0.0254 * 2/ 10/2/Math.PI/2*4096; //*6.5/4.25, 4.25/2 
        }

        private double ticksPer100msToVknot(double ticksPer100ms) {
            return ticksPer100ms * 0.0254/4096*2*Math.PI*2*10/2; //*2/4.25
        }

        public double getRpm() {
            return ticksPer100msToRpm(shooterTicksPer100ms);
        }

        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("Shooter RPM", ticksPer100msToRpm(shooterTicksPer100ms));
            SmartDashboard.putNumber("Shooter Ticks per 100ms", shooterTicksPer100ms);
            SmartDashboard.putNumber("Shooter V Knot", ticksPer100msToVknot(shooterTicksPer100ms));
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
            shooterTicksPer100ms = mMaster.getSelectedSensorVelocity(0);
        }
    
        @Override
        public void writePeriodicOutputs() {
        }
    
        @Override
        public boolean checkSystem() {
            return true;
        }
}
    