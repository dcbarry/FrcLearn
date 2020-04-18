package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Spinner extends Subsystem {
        private static Spinner mInstance;
        public final TalonSRX mMaster;
        private final I2C.Port i2cPort = I2C.Port.kOnboard;
        private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
        private Color detectedColor;
        private final ColorMatch m_colorMatcher = new ColorMatch();
        private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
        private String colorString, mPrevColor = "Red";
        private int colorCounter = 0;

        private Spinner() {
            //mMaster = new VictorSPX(Constants.kIntakeMasterId);
            mMaster = new TalonSRX(Constants.kSpinnerMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(false);
            mMaster.setNeutralMode(NeutralMode.Brake);
            m_colorMatcher.addColorMatch(kBlueTarget);
            m_colorMatcher.addColorMatch(kGreenTarget);
            m_colorMatcher.addColorMatch(kRedTarget);
            m_colorMatcher.addColorMatch(kYellowTarget);    
        }
    
        public synchronized static Spinner getInstance() {
            if (mInstance == null) {
                mInstance = new Spinner();
            }
            return mInstance;
        }

        public void setPower(double power) {
            mMaster.set(ControlMode.PercentOutput, power);
        }

        public void spinClockWise() {
            setPower(1.0);
        }

        public void spinCounterClockWise() {
            setPower(-1.0);
        }

        public void rotationControl() {
            if (mPrevColor != getColor()) {
                if (getColor() == "Red") {
                    colorCounter++;
                }
                mPrevColor = getColor();
            }
            if (colorCounter != 4)
                spinClockWise();
            else
                stop();
        }

        public void positionControl() {
            String gameData;
            gameData = DriverStation.getInstance().getGameSpecificMessage();
            if (gameData.length() > 0)
            {
                double power = 0.5;
                switch (gameData.charAt(0))
                {
                    case 'B' :
                        if (getColor() == "Red")
                            power = 0;
                    break;
                    case 'G' :
                        if (getColor() == "Yellow")
                            power = 0;
                    break;
                    case 'R' :
                        if (getColor() == "Blue")
                            power = 0;
                    break;
                    case 'Y' :
                        if (getColor() == "Green")
                            power = 0;
                    break;
                    default :
                        power = 0;
                    break;
                }
                setPower(power);
            } else {
                rotationControl();
            }
        }

        private String getColor() {
            detectedColor = m_colorSensor.getColor();
            
            ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kBlueTarget) {
                colorString = "Blue";
            } else if (match.color == kRedTarget) {
                colorString = "Red";
            } else if (match.color == kGreenTarget) {
                colorString = "Green";
            } else if (match.color == kYellowTarget) {
                colorString = "Yellow";
            } else {
                colorString = "Unknown";
            }

            return colorString;
        }

        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("Red", detectedColor.red);
            SmartDashboard.putNumber("Green", detectedColor.green);
            SmartDashboard.putNumber("Blue", detectedColor.blue);
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
    