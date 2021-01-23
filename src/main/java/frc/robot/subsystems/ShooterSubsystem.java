package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_DigitalInput;
import frc.robot.motors.ShooterMotor;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterMotor m_shooterMotor;
    private CT_DigitalInput m_cellInput;
    private Solenoid m_bopper;
    private VisionSubsystem m_visionSubsystem;
    private AcquisitionSubsystem m_acquisitionSubsystem;
    private StorageSubsystem m_storageSubsystem;
    private boolean m_isShooting;
    private boolean m_isRunningShooterMotor;

    public ShooterSubsystem(StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem,
            AcquisitionSubsystem acquisitionSubsystem) {
        register();

        m_visionSubsystem = visionSubsystem;
        m_acquisitionSubsystem = acquisitionSubsystem;
        m_storageSubsystem = storageSubsystem;

        m_shooterMotor = new ShooterMotor();
        m_bopper = new Solenoid(Constants.PCM_CAN_ID, Constants.BOPPER_PCM_PORT);

        m_isShooting = false;
        m_isRunningShooterMotor = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getSpeed());
        // SmartDashboard.putBoolean("Is Shooter Slot Occupied", !m_cellInput.get());
        SmartDashboard.putBoolean("Is Robot Shooting", m_isShooting);
    }

    /**
     * Returns the opposite value of the getter for the sensor as for example if the
     * getter returns true that means the sensor is not blocked.
     * 
     * @return if the shooter slot is occupied by a powercell
     */
    public boolean isShooterBallOccupied() {
        return !m_cellInput.get();
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
    public boolean getIsShooting() {
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
     * Starts the shooter motor, also sets the variable isShooting in the storage
     * subsystem and in the shooter subsystem to true
     */
    public void startShooterMotor() {
        double currentMoveSpeed = RobotContainer.getDrivebaseSubsystem().getCurrentMoveSpeedAverage();

        if (currentMoveSpeed < 0.5 && currentMoveSpeed > -0.5) { // make sure the robot is lower than half speed
            m_acquisitionSubsystem.stopAcquirerMotor();
            m_acquisitionSubsystem.deployAcquirer();
            m_isRunningShooterMotor = true;
            m_shooterMotor.start((int) m_visionSubsystem.getLidarAverage());
        } else {
            System.out.println("Robot is running to fast to start shooter motor");
        }
    }

    /**
     * Starts the shooter motor, also sets the variable isShooting in the storage
     * subsystem and in the shooter subsystem to false. Rotates the drum back to
     * acquire position.
     */
    public void stopShooterMotor() {
        // m_acquisitionSubsystem.retractAcquirer();
        m_shooterMotor.stop();
        m_isRunningShooterMotor = false;
        m_isShooting = false;
        RobotContainer.getIndexDrumCommand(m_storageSubsystem.getDrumStoragePositionInput(), false).schedule();
    }

    public boolean getIsRunningShooterMotor() {
        return m_isRunningShooterMotor;
    }

    public ShooterMotor getShooterMotor() {
        return m_shooterMotor;
    }
}