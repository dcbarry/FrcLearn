package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;
    private Shooter mShooter = Shooter.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private Intake mIntake = Intake.getInstance();

    private double mShooterSetpoint;
    private boolean whetherAim = true, whetherHood = true;
    public double x;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    public void autoShoot() {
        /*if (whetherAim && mLimelight.getTargetExists()) {
            mShooterSetpoint = computeRPM();
            mTurret.setSetpoint(mLimelight.getTargetX() + mTurret.getCurrentDegree());
            /*if (x < 5.5 && x > 0)
                mHood.setSetpoint(50);
            else
                mHood.setSetpoint(computeLaunchingAngle());
        }*/
        if (whetherAim && mLimelight.getTargetExists()) {
            mShooterSetpoint = computeRPM();
            mTurret.setSetpoint(mLimelight.getTargetX() + mTurret.getCurrentDegree());
            if (x < 5.5 && x > 0)
                mHood.setSetpoint(50);
            else
                mHood.setSetpoint(computeLaunchingAngle());
            
        }
        mShooter.setBangBang(mShooterSetpoint);
        if (mShooter.hasReachedDesiredSpeed(mShooterSetpoint)) {
            /*if (whetherHood) {
                if (x < 5.5 && x > 0)
                    mHood.setSetpoint(50);
                else
                    mHood.setSetpoint(computeLaunchingAngle());
            }*/
            whetherAim = false;
            mHopper.inflow();
            mHopper.elevate();
            whetherHood = false;
        } 
    }

    public void startAiming(boolean ifAim) {
        if (ifAim) {
            whetherAim = true;
            whetherHood = true;
        } else {
            whetherAim = false;
            whetherHood = false;
        }
    }

    public void autoAim() {
        //if (mLimelight.getTargetExists())
            //forceAim();
        //else
            mTurret.faceTheTarget();  
    }

    public void forceAim() {
        mShooterSetpoint = computeRPM();
        mTurret.setSetpoint(mLimelight.getTargetX() + mTurret.getCurrentDegree());
        mHood.setSetpoint(computeLaunchingAngle());
    }

    public void autoIntake() {
        mIntake.intake();
        if (mHopper.getSensorValue() > 1820) {
            mHopper.stop();
        } else {
            mHopper.setEntrancePower(0.2);
            mHopper.setVerticalFeederPower(0.35);
        }

    }

    private double computeLaunchingAngle() {
        return -2.5 * x + 57.25;
    }

    private double computeRPM() {
        if (x > 17)
            return -0.3176 * Math.pow(x, 4) + 29.427 * Math.pow(x, 3) - 1004.857 * Math.pow(x, 2) + 15047.57 * x - 78000.0;
        else
            return -0.90685 * Math.pow(x, 3) + 35.0691 * Math.pow(x, 2) - 239.8202 * x + 3712; // combined optimization 3708
        //return -0.83417 * Math.pow(x, 3) + 32.410534 * Math.pow(x, 2) - 207.881554 * x + 3589.59; // only based on optimization
        //return -0.9965034965 * Math.pow(x, 3) + 38.68006993 * Math.pow(x, 2) - 285.9047203 * x + 3888.0; //cubic
        //return 5.795454545 * Math.pow(x, 2) + 61.22727273 * x + 2721.4375; //quad
    }

    private double computeVknot() { // m/s
        return Math.sqrt(2.0*(98.0-24.0)*0.0254*9.8)/Math.sin(Math.toRadians(computeTheta())); //+6.5
    }
    private double computeTheta() {
        return Math.toDegrees(Math.atan(2.0*(98.0-24.5) / (mLimelight.getEstimatedDistance()+30.0)));
       // return 81.5 - Math.toDegrees(Math.atan((mLimelight.getEstimatedDistance()+30) / (2*(98-24.5)))); // dont need to convert to meters because of tan()
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putNumber("shooting desirable V knot", computeVknot());
        //SmartDashboard.putNumber("shooting desirable rpm", mShooter.vknotToTicksPer100ms(computeVknot())*600/4096);
        //SmartDashboard.putNumber("shooting desirable theta", computeTheta());
        SmartDashboard.putNumber("shooting desirable new RPM", computeRPM());
        SmartDashboard.putNumber("shooting desirable new Launching angle", computeLaunchingAngle());
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    x = (mLimelight.getEstimatedDistance()+1.5) / 12;
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}