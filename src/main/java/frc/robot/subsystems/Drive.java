package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.planners.DriveMotionPlanner;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static final int kVelocityControlSlot = 0;
    private static final int kPositionControlSlot = 1;
    private static final double DRIVE_ENCODER_PPR = 2048.;
    private static final double DRIVE_GEAR_RATIO = 9.2;
    private static Drive mInstance = new Drive();
    
    // Hardware
    private final TalonFX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    //private TalonSRX mGyroDummy;
    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private double mLeftDemand;
    private double mRightDemand;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    public boolean mOverrideTrajectory = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private void configureMaster(TalonFX talon, boolean right) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);
        talon.setInverted(right);
        //talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.03, 0);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = new TalonFX(Constants.kLeftDriveMasterId);
        configureMaster(mLeftMaster, false);
        //mLeftMaster.setSensorPhase(false);

        mLeftSlave = new TalonFX(Constants.kLeftDriveSlaveId);
        mLeftSlave.follow(mLeftMaster);
        mLeftSlave.setInverted(false);

        mRightMaster = new TalonFX(Constants.kRightDriveMasterId);
        configureMaster(mRightMaster, true);
        //mRightMaster.setSensorPhase(false);
        
        mRightSlave = new TalonFX(Constants.kRightDriveSlaveId);
        mRightSlave.follow(mRightMaster);
        mRightSlave.setInverted(true);

        reloadGains();

        //mGyroDummy = new TalonSRX(Constants.kGyroDummyId);
        mPigeon = new PigeonIMU(Spinner.getInstance().mMaster);
        mRightSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);
        //mPigeon.enterCalibrationMode(CalibrationMode.Temperature);
        
        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance() {
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);

            System.out.println("Switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setPosition(double left, double right) {
    	if (mDriveControlState != DriveControlState.POSITION) {
    		mLeftMaster.selectProfileSlot(kPositionControlSlot, 0);
            mRightMaster.selectProfileSlot(kPositionControlSlot, 0);
            setBrakeMode(true);
            mDriveControlState = DriveControlState.POSITION;
    	}
    	mLeftDemand = inchesToTicks(left)+mLeftMaster.getSelectedSensorPosition(0);
    	mRightDemand = inchesToTicks(right)+mRightMaster.getSelectedSensorPosition(0);
    	mLeftMaster.set(ControlMode.Position, mLeftDemand);
    	mRightMaster.set(ControlMode.Position, mRightDemand);
    }
    
    public synchronized boolean isDoneWithPosition() {
    	if (Math.abs(mLeftDemand - mLeftMaster.getSelectedSensorPosition(0)) < 500 && Math.abs(mRightDemand - mRightMaster.getSelectedSensorPosition(0)) < 500)
    		return true;
    	else
    		return false;
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }


    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized double getHeadingDegree() {
        return mPigeon.getFusedHeading();
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        //SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        //SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        //SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        //SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        //SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        //SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
        //SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
        //SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", mPigeon.getFusedHeading());//getHeading().getDegrees()
        }
        //if (mCSVWriter != null) {
           // mCSVWriter.write();
        //}
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        mPigeon.setFusedHeading(0);
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    private static double rotationsToInches(double rotations) {
        return rotations / DRIVE_GEAR_RATIO * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesToTicks(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI) * DRIVE_ENCODER_PPR * DRIVE_GEAR_RATIO;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR * DRIVE_GEAR_RATIO / 10.0;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR);
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR);
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                //setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
        
        mLeftMaster.config_kP(kPositionControlSlot, 2.5, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kPositionControlSlot, 0.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kPositionControlSlot, 6.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kPositionControlSlot, 0.2, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kPositionControlSlot, 2.5, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kPositionControlSlot, 0.0, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kPositionControlSlot, 6.0, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kPositionControlSlot, 0.2, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches / 9.2; // divided by gear ratio change

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches / 9.2;

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0); //divided by 2 because of the encoder change)
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        POSITION
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}

