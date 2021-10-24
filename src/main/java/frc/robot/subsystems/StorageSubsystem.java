package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_DigitalInput;

public class StorageSubsystem extends SubsystemBase {

    private CT_DigitalInput m_shooterPositionInput;
    private Spark m_drumSparkMotor;
    private WPI_TalonSRX m_spinningBarMotor;
    private Compressor m_compressor;

    
    public StorageSubsystem(ShooterSubsystem shooterSubsystem) {
        register();

        m_drumSparkMotor = new Spark(Constants.DRUM_SPARK_PWM_ID);
        m_spinningBarMotor = new WPI_TalonSRX(Constants.SPINNING_BAR_MOTOR_CAN_ID);

        m_shooterPositionInput = new CT_DigitalInput(Constants.SHOOTER_POSITION_DIO);
        m_shooterPositionInput.setInterrupt(() -> stopDrumMotor(), false, true);
        m_compressor = new Compressor();
        

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Drum input", m_shooterPositionInput.get());
        SmartDashboard.putBoolean("Is interrupt latched", m_shooterPositionInput.isInterruptLatched());

        boolean[] drumConditions = {
            (RobotContainer.m_robotState == Constants.SHOOTING_STATE 
             || RobotContainer.m_robotState == Constants.TEST_STATE) 
        };

        // Ignore interrupts for 0.5 seconds after the dial starts moving to account for the slop in the dial.
        m_shooterPositionInput.ignoreInterruptsFor(0.35);

        // Only index the drum when the robot is either shooting or is instructed to through manual indexing (changing of the m_doIndexing variable)
        m_shooterPositionInput.onlyHandleInterruptsWhen(drumConditions);
    }

    public Compressor getCompressor() {
        return m_compressor;
    }

    public Spark getDrumMotor() {
        return m_drumSparkMotor;
    }

    /**
     * Starts the drum spark motor
     */
    public void startDrumMotor(double velocity) {
        m_drumSparkMotor.set(-velocity);
        startBarMotor();

        m_shooterPositionInput.ignoreInterruptsNow();
    }

    /**
     * Starts the drum spark motor backwards
     */
    public void startDrumMotorBackwards() {
        m_drumSparkMotor.set(Constants.DRUM_MOTOR_VELOCITY_SLOW);
    }

    /**
     * Stops the drum spark motor
     */
    public void stopDrumMotor() {
        //System.out.println("stop drum motor");
        m_drumSparkMotor.set(0);
        stopBarMotor();
    }

    /**
     * Starts the bar motor
     */
    public void startBarMotor() {
        m_spinningBarMotor.set(0.55);
    }

    /**
     * Stops the bar motor
     */
    public void stopBarMotor() {
        m_spinningBarMotor.set(0);
    }

    /**
     * Gets the shooter position digital input object
     * 
     * @return the shooter position digital input object
     */
    public CT_DigitalInput getDrumShooterPositionInput() {
        return m_shooterPositionInput;
    }
}