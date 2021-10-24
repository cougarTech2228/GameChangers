/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Toolkit.CT_CommandToggler;
import frc.robot.util.ButtonManager;
import frc.robot.util.LidarManager;
import frc.robot.util.ShooterMotor;
import frc.robot.util.TrajectoryManager;
import frc.robot.Toolkit.CT_CommandToggler.CommandState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Robot Subsystems
    private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();

    // TrajectoryManager must be instantiated after DrivebaseSubsystem since it
    // relies on it

    private final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    private final static LidarManager m_lidarManager = new LidarManager();
    private final static AcquisitionSubsystem m_acquisitionSubsystem = new AcquisitionSubsystem();
    private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final static StorageSubsystem m_storageSubsystem = new StorageSubsystem(m_shooterSubsystem);

    private final static ButtonManager m_buttonManager = new ButtonManager(m_shooterSubsystem, m_storageSubsystem, m_acquisitionSubsystem);

    public static int m_robotState = 0;

    private final static SendableChooser<Double> m_manualVelocityChooser = new SendableChooser<>();

    private static Command m_basicTrajectoryCommand;
    private static Command m_barrelRacingTrajectoryCommand;
    private static Command m_slalomTrajectoryCommand;
    private static Command m_bounceTrajectoryCommand;
    private static Command[] m_galacticSearchTrajectoryCommands;
    private static Command m_powerTowerCommand = new InstantCommand().withName("Power Tower");


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        DriverStation.getInstance().silenceJoystickConnectionWarning(true);

        // The import of trajectories from PathWeaver-generated json files can be very
        // time consuming so we're putting the Trajectory Manager into a thread.
        //Thread trajectoryManagerThread = new Thread(m_trajectoryManager);
        //trajectoryManagerThread.start();

        Thread lidarManagerThread = new Thread(m_lidarManager);
        lidarManagerThread.start();

        // Configure the button and shuffleboard bindings
        m_buttonManager.configureButtonBindings();
        configureShuffleboardBindings();
    }

    // Use this method to define shuffleboard buttons or widgets
    private void configureShuffleboardBindings() {

        // Velocity Options

        SmartDashboard.putData("Velocity Chooser", m_manualVelocityChooser);
        m_manualVelocityChooser.setDefaultOption("Use Lidar", -1.0);
        m_manualVelocityChooser.addOption("Initiation Line", 75000.0);
        m_manualVelocityChooser.addOption("Trench", 77000.0);
        m_manualVelocityChooser.addOption("Control Panel", 85000.0);
        m_manualVelocityChooser.addOption("Diagnostic Velocity", 30000.0);

        /*
        * // Command Buttons SmartDashboard.putData("Cancel all commands", new
        * InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        * SmartDashboard.putData("Reset Drum Array", new
        * PrintCommand("resetting drum array").andThen(() ->
        * m_storageSubsystem.resetBallArray()));
        * SmartDashboard.putData("Repopulate drum array", getRepopulateArrayCommand());
        * 
        * // Diagnostic buttons SmartDashboard.putData("Rotate Drum Forwards", new
        * InstantCommand(m_storageSubsystem::startDrumMotor, m_storageSubsystem));
        * SmartDashboard.putData("Rotate Drum Backwards", new
        * InstantCommand(m_storageSubsystem::startDrumMotorBackwards,
        * m_storageSubsystem)); SmartDashboard.putData("Stop Drum Motor", new
        * InstantCommand(m_storageSubsystem::stopDrumMotor, m_storageSubsystem));
        * SmartDashboard.putData("Change mode to shooting", new InstantCommand(() ->
        * m_shooterSubsystem.setIsShooting(true), m_shooterSubsystem));
        * SmartDashboard.putData("Change mode to acquiring", new InstantCommand(() ->
        * m_shooterSubsystem.setIsShooting(false), m_shooterSubsystem));
        * SmartDashboard.putData("Rotate drum one index",
        * getIndexDrumCommand(m_storageSubsystem.getDrumStoragePositionInput(), true));
        * SmartDashboard.putData("Bopper", getBopperCommand());
        * SmartDashboard.putData("Wiggle", getShakeDialCommand());
        * SmartDashboard.putData("Run Acquirer Motor", new InstantCommand(() ->
        * m_acquisitionSubsystem.startAcquirerMotor()));
        * SmartDashboard.putData("Stop Acquirer Motor", new
        * InstantCommand(m_acquisitionSubsystem::stopAcquirerMotor));
        * SmartDashboard.putData("Deploy Acquirer", new
        * InstantCommand(m_acquisitionSubsystem::deployAcquirer));
        * SmartDashboard.putData("Retract Acquirer", new
        * InstantCommand(m_acquisitionSubsystem::retractAcquirer));
        * SmartDashboard.putData("Reverse Acquirer", new
        * InstantCommand(m_acquisitionSubsystem::startAcquirerMotorReverse));
        */
    }
    
    public static Double getManualVelocity() {
        return m_manualVelocityChooser.getSelected();
    }

    // Command Getters

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //System.out.println("Auto: " + m_autoChooser.getSelected().getName());
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                m_storageSubsystem.stopDrumMotor();
                m_shooterSubsystem.getShooterMotor().start(m_shooterSubsystem);
            }),
            RobotContainer.getShootCommand(5),
            new InstantCommand(() -> m_drivebaseSubsystem.driveBackwards()),
            new WaitCommand(1),
            new InstantCommand(() -> m_drivebaseSubsystem.stop())
        );
        
        
    }

    // Shooting Commands

    public static ShootCommand getShootCommand(int numOfShots) {
        return new ShootCommand(m_shooterSubsystem, m_storageSubsystem, numOfShots);
    }

    public static BopperCommand getBopperCommand() {
        return new BopperCommand(m_shooterSubsystem);
    }

    public static RumbleCommand getRumbleCommand(double time) {
        return new RumbleCommand(time);
    }

    // Subsystem Getters

    public static DrivebaseSubsystem getDrivebaseSubsystem() {
        return m_drivebaseSubsystem;
    }

    public static AcquisitionSubsystem getAcquisitionSubsystem() {
        return m_acquisitionSubsystem;
    }

    public static StorageSubsystem getStorageSubsystem() {
        return m_storageSubsystem;
    }

    public static ShooterSubsystem getShooterSubsystem() {
        return m_shooterSubsystem;
    }

    public static VisionSubsystem getVisionSubsystem() {
        return m_visionSubsystem;
    }

    public static LidarManager getLidarManager() {
        return m_lidarManager;
    }

    // public static LidarSubsystem getLidarSubsystem() {
    //     return m_lidarSubsystem;
    // }

    public static void setBasicTrajectoryCommand(CommandBase command) {
        m_basicTrajectoryCommand = command.withName("Basic");
    }

    public static void setBarrelRacingTrajectoryCommand(CommandBase command) {
        m_barrelRacingTrajectoryCommand = command.withName("Barrel Racing");
    }

    public static void setSlalomTrajectoryCommand(CommandBase command) {
        m_slalomTrajectoryCommand = command.withName("Slalom");
    }

    public static void setBounceTrajectoryCommand(CommandBase command) {
        m_bounceTrajectoryCommand = command.withName("Bounce");
    }

    public static void setGalacticSearchTrajectoryCommands(CommandBase[] commands) {
        // m_galacticSearchTrajectoryCommand = command.withName("Galactic Search").beforeStarting(() -> {
        //   m_acquisitionSubsystem.deployAcquirer();
        //   m_acquisitionSubsystem.startAcquirerMotor();
        // });
        m_galacticSearchTrajectoryCommands = commands;
    }

    public static Command[] getGalacticSearchTrajectoryCommands() {
        return m_galacticSearchTrajectoryCommands;
    }

    public static void configureAutoChooser() {
      //SmartDashboard.putData("Auto Chooser", m_autoChooser);
    //   m_autoChooser.setDefaultOption("Barrel Race", m_barrelRacingTrajectoryCommand);
    //   m_autoChooser.addOption("Slalom", m_slalomTrajectoryCommand);
    //   m_autoChooser.addOption("Bounce", m_bounceTrajectoryCommand);
    //   m_autoChooser.addOption("Galactic Search", new GalacticSearchAutoCommand(m_visionSubsystem)
    //   .beforeStarting(() -> {
    //     m_acquisitionSubsystem.deployAcquirer(true);
    //     m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_SLOW);
    //   }
    //   ).withName("Galactic Search"));
      //m_autoChooser.addOption("Power Tower", m_powerTowerCommand);
      //m_autoChooser.setDefaultOption("Basic", m_basicTrajectoryCommand);
      //m_autoChooser.addOption("GS-Red-A", m_galacticSearchTrajectoryCommands[2]);
      //m_autoChooser.addOption("GS-Red-B", m_galacticSearchTrajectoryCommands[3]);
      //m_autoChooser.addOption("GS-Blue-A", m_galacticSearchTrajectoryCommands[0]);
      //m_autoChooser.addOption("GS-Blue-B", m_galacticSearchTrajectoryCommands[1]);
    }
  
    // public static String getAutoChooserOption(){
    //   Command selectedCommand = m_autoChooser.getSelected();
    //   if(selectedCommand != null){
    //     return selectedCommand.getName();
    //   }
    //   return null;
    // }
}