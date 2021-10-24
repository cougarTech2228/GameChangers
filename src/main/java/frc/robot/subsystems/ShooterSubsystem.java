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
    private boolean m_isMotorUpToSpeed;
    public int m_targetDistance;

    public ShooterSubsystem() {
        register();

        m_shooterMotor = new ShooterMotor(this);
        m_bopper = new Solenoid(Constants.PCM_CAN_ID, Constants.BOPPER_PCM_PORT);

        m_isMotorUpToSpeed = false;
        m_targetDistance = 10;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Distance", m_targetDistance);
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Calculated velocity", m_shooterMotor.getVelocity());
        SmartDashboard.putNumber("Lidar Distance: ", RobotContainer.getLidarManager().getLidarAverage());
        SmartDashboard.putBoolean("Is shooter motor up to speed", m_isMotorUpToSpeed);

        m_isMotorUpToSpeed = false;
        if(RobotContainer.m_robotState == Constants.SHOOTING_STATE) {
            if((m_shooterMotor.getSelectedSensorVelocity() + 4000) >= m_shooterMotor.getVelocity()) {
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
    // public boolean isShooting() {
    //     return m_isShooting;
    // }

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
        //RobotContainer.getStorageSubsystem().getCompressor().start();
        m_shooterMotor.stopMotor();
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