package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousGroup extends SequentialCommandGroup {

    public AutonomousGroup(DriveSubsystem s_drive, CameraSubsystem s_camera) {
        addRequirements(s_drive);
        addRequirements(s_camera);
        addCommands(/* TODO HERE */);
    }

}