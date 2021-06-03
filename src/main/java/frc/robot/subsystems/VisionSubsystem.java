package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
    private String selectedAuto = null;
    NetworkTable visionModeTable = NetworkTableInstance.getDefault().getTable("Vision Mode");
    NetworkTableEntry selected = visionModeTable.getEntry("selected");
    ShuffleboardTab tab = Shuffleboard.getTab("PowerTower");
    NetworkTableEntry xOffset = tab.add("PT Offset", -30).getEntry();

    public VisionSubsystem() {
        register();
        
    }

    @Override
    public void periodic() {
        String selection = "Power Tower";
        
        
        if(selection != null &&
            !selection.equals (selectedAuto)) {
                
                System.out.println(selection);
                selectedAuto = selection;
                selected.setString(selection);

        }
    }
}
