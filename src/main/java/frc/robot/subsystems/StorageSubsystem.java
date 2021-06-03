package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_DigitalInput;

public class StorageSubsystem extends SubsystemBase {

    private CT_DigitalInput m_drumShooterPositionInput;
    private Spark m_drumSparkMotor;
    private WPI_TalonSRX m_spinningBarMotor;
    private ShooterSubsystem m_shooterSubsystem;
    private boolean m_doIndexing;
    private Compressor m_compressor;
    
    public StorageSubsystem(ShooterSubsystem shooterSubsystem) {
        register();

        m_drumSparkMotor = new Spark(Constants.DRUM_SPARK_PWM_ID);
        m_spinningBarMotor = new WPI_TalonSRX(Constants.SPINNING_BAR_MOTOR_CAN_ID);

        m_drumShooterPositionInput = new CT_DigitalInput(Constants.SHOOTER_POSITION_DIO);
        m_drumShooterPositionInput.setInterrupt(() -> stopDrumMotor(), false, true);
        m_compressor = new Compressor();
        
        m_doIndexing = false;
        m_shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void periodic() {
        boolean[] drumConditions = {
            (m_shooterSubsystem.isShooting() || m_doIndexing) 
        };

        // Only index the drum when the robot is either shooting or is instructed to through manual indexing (changing of the m_doIndexing variable)
        m_drumShooterPositionInput.onlyHandleInterruptsWhen(drumConditions);
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
        //System.out.println("start drum motor");
        m_drumSparkMotor.set(-velocity);
        startBarMotor();
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
        doIndexing(false);
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
     * Setter method for the m_doIndexing variable which will decide whether or not the dial will index
     * @param value the value to be set to m_doIndexing
     */
    public void doIndexing(boolean value) {
        //System.out.println("Changing doIndexing from: " + m_doIndexing + "to: " + value);
        m_doIndexing = value;
        
    }

    /**
     * Gets the shooter position digital input object
     * 
     * @return the shooter position digital input object
     */
    public CT_DigitalInput getDrumShooterPositionInput() {
        return m_drumShooterPositionInput;
    }
}