package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMotor extends WPI_TalonSRX {

    private boolean m_encodersAreAvailable;
    private NetworkTableEntry m_velocityEntry;
    private ShooterSubsystem m_shooterSubsystem;

    public ShooterMotor(ShooterSubsystem shooterSubsystem) {
        
        super(Constants.SHOOTER_CAN_ID);

        m_shooterSubsystem = shooterSubsystem;
        
        m_velocityEntry = Shuffleboard.getTab("Shooter Velocity Adjuster").add("Shooter Velocity", 1).getEntry();
        
        configFactoryDefault();

        m_encodersAreAvailable = configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PID_PRIMARY, Constants.kTimeoutMs) == ErrorCode.OK;
        
        config_kP(0, 0.01764, Constants.kTimeoutMs); //.0465
        config_kI(0, 0, Constants.kTimeoutMs);
        config_kD(0, 0, Constants.kTimeoutMs);
        config_kF(0, 0.00851, Constants.kTimeoutMs);
        config_IntegralZone(0, 0, Constants.kTimeoutMs);
        configClosedLoopPeakOutput(0, 1.0, Constants.kTimeoutMs);
        configAllowableClosedloopError(0, 0, Constants.kTimeoutMs);
        setInverted(true);
        setSensorPhase(false);
        configVoltageCompSaturation(11);

        configClosedLoopPeriod(0, 1, Constants.kTimeoutMs);

        configPeakOutputForward(+1.0, Constants.kTimeoutMs);
        configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

        configPeakCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
		configPeakCurrentDuration(Constants.SHOOTER_CURRENT_DURATION);
		configContinuousCurrentLimit(Constants.SHOOTER_CONTINUOUS_CURRENT_LIMIT);
		enableCurrentLimit(true);

        System.out.println("Are encoders available for shooter motor? " + m_encodersAreAvailable);
    }

    /**
     * Start the shooter motor
     */
    public void start(ShooterSubsystem shooterSubsystem) {
        RobotContainer.getStorageSubsystem().getCompressor().stop();
        double velocity = getVelocity();
        System.out.println("Setting velocity to: " + velocity);
        
        set(ControlMode.Velocity, velocity);

        // Shuffleboard velocity
        //set(ControlMode.Velocity, m_velocityEntry.getDouble(1));
    }

    public double getFormulaVelocity() {
        double distance = RobotContainer.getLidarManager().getLidarAverage();

        if(distance < 275 && distance > 25) { // Arbitrary values that will probably have to be adjusted
            double velocity = (-0.029 * distance * distance * distance) + (15.725 * distance * distance) - (2683.6 * distance) + 209603; 
            return velocity;
        } else {
            return 0;
        }
        
    }

    public double getVelocity() {
        int distance = m_shooterSubsystem.m_targetDistance;

            switch(distance) {
                case 10: return 67156;
                case 15: return 66556;
                case 20: return 72840;
                case 25: return 82500;
                default: System.out.println("Incorrect distance created"); return 0; // Keep this 0 
            }
    }
}