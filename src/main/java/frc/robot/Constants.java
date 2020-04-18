package frc.robot;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01; //0.01

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 23.5;
    public static final double kDriveWheelDiameterInches = 6.25;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 10.0;  // N*m / (rad/sec) TODO tune was 12
    public static final double kDriveVIntercept = 0.106;  // V
    public static final double kDriveKv = 0.0519;  // V per rad/s
    public static final double kDriveKa = 0.00703;  // V per rad/s^2

    /* CONTROL LOOP GAINS */

    // Mechanical constants.
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.2; //0.9 i suggest 0.18
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 1.0; //10 or 0
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // PID gains for shooter
    // Units: setpoint, error, and output are in native units per 100ms.

    public static final double kShooterKp = 5.0;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 30.0;
    public static final double kShooterKf = 0.02; // lower speed:  0.06;
    public static final double kShooterEpsilon = 1.0;//33000;

    // Turret gains
    public static final double kTurretKp = 1.1;
    public static final double kTurretKi = 0.0;
    public static final double kTurretKd = 18.0;
    public static final double kTurretKf = 0.0;

    // Hood gains
    public static final double kHoodKp = 4.2;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 28.0;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1TJjk_zeRJfgE3Ou8_LwR-5cxomrNaOS92bc7FUiBrkQ/edit#gid=0

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 10; //use for constructors

    // Drive
    public static final int kLeftDriveMasterId = 9;
    public static final int kLeftDriveSlaveId = 8;
    public static final int kRightDriveMasterId = 6;
    public static final int kRightDriveSlaveId = 7;
    //public static final int kGyroDummyId = 0; 

    // Shooter
    public static final int kShooterMasterId = 13;
    public static final int kShooterSlaveId = 14;

    // Turret
    public static final int kTurretMasterId = 10;

    // Hood
    public static final int kHoodMasterId = 1;

    // Intake
    public static final int kIntakeMasterId = 2;

    // Hopper
    public static final int kHopperMasterId = 4;
    public static final int kHopperSlaveId = 5;
    public static final int kHopperVerticalMasterId = 3;

    // Climber
    public static final int kClimberMasterId = 12;
    public static final int kClimberSlaveId = 11;

    public static final int kSpinnerMasterId = 0;

    // Solenoids
    public static final int kIntakeSolenoidId = 1;
    public static final int kClimberSolenoidId = 0;

    // Sensors
    public static final int kHopperDistanceSensor = 3;
}