// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class ShootCommand extends CommandBase {
  /** Creates a new ShootCommand. */

  private StorageSubsystem m_storageSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private int m_timesShot;
  private boolean m_isBopping;
  private int m_numOfShots;

  public ShootCommand(ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, int numOfShots) {
    m_storageSubsystem = storageSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_numOfShots = numOfShots;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setIsShooting(true);
    m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_FAST);
    m_timesShot = 0;
    m_isBopping = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooterSubsystem.isShooterMotorUpToSpeed()) {
      if(!m_isBopping && m_storageSubsystem.getDrumMotor().getSpeed() == 0) {
        //System.out.println("Running the thing");
        m_isBopping = true;
        new SequentialCommandGroup(
          RobotContainer.getBopperCommand(),//.andThen(() -> m_isBopping = false),
          new WaitCommand(0.3)
          .andThen(() -> {
            m_isBopping = false;
            m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_FAST);
            m_timesShot++;
          })
        ).schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setIsShooting(false);
    m_storageSubsystem.stopDrumMotor();
    m_shooterSubsystem.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timesShot == m_numOfShots) {
      return true;
    }
    return false;
  }
}
