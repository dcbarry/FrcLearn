package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TortoDriveHelper;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.*;
import frc.robot.loops.Looper;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private TortoDriveHelper mTortoDriveHelper = new TortoDriveHelper();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Limelight.getInstance(),
                    Drive.getInstance(),
                    Superstructure.getInstance(),
                    Intake.getInstance(),
                    Shooter.getInstance(),
                    Turret.getInstance(),
                    Hood.getInstance(),
                    Hopper.getInstance(),
                    Climber.getInstance(),
                    Hopper.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private Climber mClimber = Climber.getInstance();
    private Spinner mSpinner = Spinner.getInstance();
    private Joystick mDriveStick = new Joystick(0);
    private Joystick mOperatorStick = new Joystick(1);
    private Joystick mTestStick = new Joystick(2);
    private Compressor mCompressor = new Compressor();

    private AutoModeExecutor mAutoModeExecutor;
    private SendableChooser mAutoModeChooser;
    private boolean climberRetracted, mIfShooting;
    private double mThrottleLimit = 1;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    private enum AutoMode {
        DRIVE_STRAIGHT, SAFE_TRENCH, GREEDY, EIGHT_OUR_TRENCH, TEST
    }

    @Override
    public void robotInit() {
        try {
            mAutoModeChooser = new SendableChooser<>();
            mAutoModeChooser.setDefaultOption("Safe 8 balls from enemy trench", AutoMode.SAFE_TRENCH); // need to change this
            mAutoModeChooser.addOption("8 balls from our trench", AutoMode.EIGHT_OUR_TRENCH);
            mAutoModeChooser.addOption("Greedy 8 balls from enemy trench", AutoMode.GREEDY);
            mAutoModeChooser.addOption("Cross Auto line", AutoMode.DRIVE_STRAIGHT);
            SmartDashboard.putData("Auto Modes", mAutoModeChooser);
            mAutoModeExecutor = new AutoModeExecutor();

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setBrakeMode(false);
            mDrive.zeroSensors();
            mTrajectoryGenerator.generateTrajectories();
            mLimelight.setLEDLight(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDisabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mCompressor.stop();
            
            Drive.getInstance().setBrakeMode(true);
            Drive.getInstance().zeroSensors();
            mLimelight.setLEDLight(true);

            if (mAutoModeChooser.getSelected() == AutoMode.GREEDY)
                mAutoModeExecutor.setAutoMode(new GreedyOppoTrench(false));
            else if (mAutoModeChooser.getSelected() == AutoMode.SAFE_TRENCH)
                mAutoModeExecutor.setAutoMode(new SafeOppoTrench(false));
            else if (mAutoModeChooser.getSelected() == AutoMode.EIGHT_OUR_TRENCH)
                mAutoModeExecutor.setAutoMode(new EightBallOurTrench(false));
            else
                mAutoModeExecutor.setAutoMode(new TestMode(false));

            mAutoModeExecutor.start();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mCompressor.start();
            mCompressor.setClosedLoopControl(true);
            mEnabledLooper.start();
            mDrive.setBrakeMode(false);
            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            //mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            System.out.println("Starting check systems.");
            mDisabledLooper.stop();
            mEnabledLooper.stop();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        outputToSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        double throttle = mDriveStick.getRawAxis(1);
        double turn = mDriveStick.getRawAxis(4);
        
        //mShooter.setOpenLoop(0.6);
        //mShooter.setBangBang(6500); //
        //mHood.setSetpoint(50.0); //
        //mHood.setOpenLoop(-mOperatorStick.getRawAxis(1));
        if (mOperatorStick.getRawButton(8))
            mSuperstructure.autoIntake();
        else if (mOperatorStick.getRawButton(7))
            mIntake.outtake();
        else if (mOperatorStick.getRawButton(5)) {
            //mLimelight.setLEDLight(true);
            mSuperstructure.autoShoot();
            //mIntake.retract();
            //mHopper.inflow();
            //mHopper.elevate();
            mIfShooting = true;
        } else if (mOperatorStick.getRawButton(6)) {
            mSuperstructure.autoAim();     
            mShooter.setOpenLoop(0.4);
        } else if (mDriveStick.getRawAxis(3) > 0.4)
            mIfShooting = true;
        else if (mOperatorStick.getRawButton(4)) {
            mShooter.setOpenLoop(-1);
            mHopper.setVerticalFeederPower(-1);
            mHopper.setEntrancePower(-0.4);
        } else if (mOperatorStick.getRawButton(3))
            mIntake.setPower(-1);
        else {
            //mLimelight.setLEDLight(true);
            mHopper.stop();
            mShooter.stop();
            mIntake.stop();
            mHood.setSetpoint(35);
            mSuperstructure.startAiming(true);
            mIfShooting = false;
        }

        
        if (mOperatorStick.getPOV(0) == 0) 
            mTurret.setSetpoint(0);
        else if (mOperatorStick.getPOV(0) == 45) 
            mTurret.setSetpoint(45);
        else if (mOperatorStick.getPOV(0) == 90) 
            mTurret.setSetpoint(90);
        else if (mOperatorStick.getPOV(0) == 135) 
            mTurret.setSetpoint(135);
        else if (mOperatorStick.getPOV(0) == 180) 
            mTurret.setSetpoint(180);
        else if (mOperatorStick.getPOV(0) == 225) 
            mTurret.setSetpoint(225);
        else if (mOperatorStick.getPOV(0) == 270) 
            mTurret.setSetpoint(270);
        else if (mOperatorStick.getPOV(0) == 315) 
            mTurret.setSetpoint(-45);

        if (mOperatorStick.getRawButton(10)) {
            mClimber.foldout();
            mTurret.setSetpoint(180);
            mThrottleLimit = 0.7;
            climberRetracted = true;
        } else if (mOperatorStick.getRawButton(9)) {
            mClimber.retract();
            mThrottleLimit = 1;
            climberRetracted = false;
        }
        if (climberRetracted) {
            mClimber.setPower(mOperatorStick.getRawAxis(1));
            if (mOperatorStick.getRawButton(2))
                mSpinner.positionControl();
            else
                mSpinner.setPower(mOperatorStick.getRawAxis(2));
        }

        if (mIfShooting)
            mDrive.setVelocity(new DriveSignal(0,0), new DriveSignal(0, 0));
        else 
            mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-mThrottleLimit * throttle, turn, true, true));

        //outputToSmartDashboard();
    }

    @Override
    public void testPeriodic() {
    }

    private void outputToSmartDashboard() {
        //RobotState.getInstance().outputToSmartDashboard();
        mDrive.outputTelemetry();
        mLimelight.outputTelemetry();
        mTurret.outputTelemetry();
        mShooter.outputTelemetry();
        mHood.outputTelemetry();
        mHopper.outputTelemetry();
        mSuperstructure.outputTelemetry();
        SmartDashboard.putNumber("small stick", mDriveStick.getRawAxis(2));
        SmartDashboard.putNumber("big stick", mTestStick.getRawAxis(0));
        //Intake.getInstance().outputTelemetry();
        //mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}
