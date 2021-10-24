package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AcquisitionSubsystem extends SubsystemBase {

    private CANSparkMax m_acquisitionMotor;
    private Solenoid m_acquirerExtender;
    private boolean m_isAcquirerDeployed;

    public AcquisitionSubsystem() {
        register();

        m_acquisitionMotor = new CANSparkMax(42, MotorType.kBrushless);
        m_acquirerExtender = new Solenoid(Constants.PCM_CAN_ID, Constants.ACQUIRER_DEPLOY_PCM_PORT);
        m_isAcquirerDeployed = false;
    }

    @Override
    public void periodic() {

    }
    
    /**
     * Deploys the acquirer by setting the solenoid to true
     */
    public void deployAcquirer(boolean runAcquirer) {
        m_acquirerExtender.set(true);
        m_isAcquirerDeployed = true;
        if(runAcquirer)
            m_acquisitionMotor.set(-Constants.ACQUIRER_MOTOR_SPEED); 
    }

    /**
     * Retracts the acquirer by setting the solenoid to false
     */
    public void retractAcquirer() {
        m_acquisitionMotor.set(0);
        m_acquirerExtender.set(false);
        m_isAcquirerDeployed = false;
    }

    /**
     * Starts the acquirer motor
     */
    public void startAcquirerMotor() {
        m_acquisitionMotor.set(-Constants.ACQUIRER_MOTOR_SPEED);
    }

    /**
     * Stops the acquirer motor
     */
    public void stopAcquirerMotor() {
        m_acquisitionMotor.set(0);
    }

    /**
     * Runs the acquirer motor in reverse
     */
    public void startAcquirerMotorReverse() {
        m_acquisitionMotor.set(Constants.ACQUIRER_MOTOR_SPEED);
    }

    public boolean isAcquirerDeployed() {
        return m_isAcquirerDeployed;
    }
}