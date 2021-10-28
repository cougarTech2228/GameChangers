package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.OI;

/**
 * RumbleCommand
 * 
 * Rumbles the controller based on the amount of time passed in
 * 
 * This will be used to tell the driver that a button push will be invalidated for reasons:
 * -Started to shoot the entire drum when the motor hasn't spun up
 * -The distance of the robot was not in between the max and min distances
 */
public class RumbleCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public RumbleCommand(double time) {
        addCommands (
            new InstantCommand(() -> OI.setXboxRumbleSpeed(Constants.XBOX_RUMBLE_SPEED)),
            new WaitCommand(time)
            .andThen(() -> OI.setXboxRumbleStop())
        );
    }
}