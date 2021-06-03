package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AcquisitionSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TargetCorrectionCommand extends CommandBase {
    // The subsystem the command runs on
    private final DrivebaseSubsystem m_drivebaseSubsystem;
    private final AcquisitionSubsystem m_acquisitionSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntinst.getTable("PowerTower");

    private NetworkTableEntry difference = table.getEntry("difference");

    private boolean matchedLines;

    public TargetCorrectionCommand(DrivebaseSubsystem drivebaseSubsystem, AcquisitionSubsystem acuisitionSubsystem, ShooterSubsystem shooterSubsystem) {
        m_drivebaseSubsystem = drivebaseSubsystem;
        m_acquisitionSubsystem = acuisitionSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_drivebaseSubsystem);
    }
  
    @Override
    public void initialize() {
        RobotContainer.getStorageSubsystem().getCompressor().stop();
        System.out.println("Target Correction Started");
        m_drivebaseSubsystem.setBrakeMode();
        m_acquisitionSubsystem.stopAcquirerMotor();
        matchedLines = false;
    }
  
    @Override
    public boolean isFinished() {
        difference = table.getEntry("difference");
        
        double diff = difference.getDouble(-1000);
        //System.out.println("Outside Difference: " + diff);

        double magnitude = Math.abs(diff);
        
        if (magnitude < Constants.TC_UPPER_MAGNITUDE_BOUND) { // Essentially confirms that the line is actually on the screen
            m_drivebaseSubsystem.allowDriving(false);

            if(magnitude > Constants.TC_LOWER_MAGNITUDE_BOUND) { // If the magnitude is between 480 and 5 the robot turns fast
                if(diff > 0) {
                    //System.out.println("Turn left fast");
                    m_drivebaseSubsystem.turnLeft(Constants.TC_TURN_FAST_SPEED);
                } else {
                    //System.out.println("Turn right fast");
                    m_drivebaseSubsystem.turnRight(Constants.TC_TURN_FAST_SPEED);
                }
            } else {  //If the magnitude is less than 5

                if (diff > 1 && !matchedLines) { 
                    //System.out.println("Turn left slow");
                    m_drivebaseSubsystem.turnLeft(Constants.TC_TURN_SLOW_SPEED);
                }
                else if(diff < 1 && !matchedLines) {
                    //System.out.println("Turn right slow");
                    m_drivebaseSubsystem.turnRight(Constants.TC_TURN_SLOW_SPEED);
                }
                else { // Lines are matched up
                    matchedLines = true;
                    m_drivebaseSubsystem.stop();

                    if(!RobotContainer.getShooterSubsystem().isShooting()) { // Start the motor if it is not already started
                        System.out.println("Starting the shooter motor");
                        m_shooterSubsystem.startShooterMotor();
                    }

                    if(RobotContainer.getShooterSubsystem().isShooterMotorUpToSpeed()) {
                        new SequentialCommandGroup(
                            new WaitCommand(0.25),
                            RobotContainer.getShootCommand(5)
                        ).schedule();
                        return true;
                    } else {
                        //System.out.println("Waiting for shooter motor to come up to speed");
                    }
                    
                }
                
            }
        } else {
            //System.out.println("Outside Difference: " + diff);
        }

      return false;
    }

    @Override
    public void end(boolean interuppted){
        //m_drivebaseSubsystem.stop();
        m_drivebaseSubsystem.setCoastMode();
        m_drivebaseSubsystem.allowDriving(true);
    }
  }