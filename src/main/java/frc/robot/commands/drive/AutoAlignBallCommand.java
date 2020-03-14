package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignBallCommand extends CommandBase {

    private DriveSubsystem s_drive;

    public AutoAlignBallCommand(DriveSubsystem s_drive) {
        this.s_drive = s_drive;
    }

    

}