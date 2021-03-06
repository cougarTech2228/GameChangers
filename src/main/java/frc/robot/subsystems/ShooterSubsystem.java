package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMotor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterMotor m_shooterMotor;
    private Solenoid m_bopper;
    private boolean m_isShooting;
    private boolean m_isMotorUpToSpeed;

    public ShooterSubsystem() {
        register();

        m_shooterMotor = new ShooterMotor();
        m_bopper = new Solenoid(Constants.PCM_CAN_ID, Constants.BOPPER_PCM_PORT);

        m_isShooting = false;
        m_isMotorUpToSpeed = false;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean("Is Robot Shooting", m_isShooting);
        SmartDashboard.putNumber("Lidar Distance: ", RobotContainer.getLidarManager().getLidarAverage());

        if(m_isShooting) { // + 4000 is if the sensor is a bit low, this is mainly to prevent the driver from immediately starting it
            if((m_shooterMotor.getSelectedSensorVelocity() + 4000) >= m_shooterMotor.getFormulaVelocity()) {
                m_isMotorUpToSpeed = true;
            } else {
                m_isMotorUpToSpeed = false;
            }
        }

        //System.out.println("Is shooter motor up to speed? " + m_isMotorUpToSpeed);
    }

    public boolean isShooterMotorUpToSpeed() {
        return m_isMotorUpToSpeed;
    }

    /**
     * Raises the solenoid which pushes the powercell into the shooter motor
     */
    public void raiseBopper() {
        m_bopper.set(true);
    }

    /**
     * Lowers the solenoid
     */
    public void lowerBopper() {
        m_bopper.set(false);
    }

    /**
     * Returns the boolean isShooting which determines the state of the drum slot
     * location
     * 
     * @return boolean isShooting
     */
    public boolean isShooting() {
        return m_isShooting;
    }

    /**
     * Sets isShooting to the passed in value
     * 
     * @param isShooting
     */
    public void setIsShooting(boolean isShooting) {
        System.out.println("Setting is shooting to: " + isShooting);
        m_isShooting = isShooting;
    }

    /**
     * Starts the shooter motor and runs the velocity adjustment command
     */
    public void startShooterMotor() {
        m_shooterMotor.start(this);
        //m_isShooting = true;
    }

    /**
     * Starts the shooter motor, also sets the variable isShooting in the storage
     * subsystem and in the shooter subsystem to false. Rotates the drum back to
     * acquire position.
     */
    public void stopShooterMotor() {
        m_shooterMotor.stopMotor();
        m_isShooting = false;
    }

    /**
     * Gets the shooter motor
     * 
     * @return the shooter motor
     */
    public ShooterMotor getShooterMotor() {
        return m_shooterMotor;
    }
}