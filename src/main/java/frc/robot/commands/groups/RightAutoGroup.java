package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveFowardCommand;
import frc.robot.commands.drive.RotateToHeadingProfiledCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RightAutoGroup extends SequentialCommandGroup{

    private final DriveSubsystem s_drive;
    private final IntakeSubsystem s_intake;
    private final ShooterSubsystem s_shooter;

    public RightAutoGroup(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter){
        this.s_drive = drive;
        this.s_intake = intake;
        this.s_shooter = shooter;
        addCommands(new RotateToHeadingProfiledCommand(s_drive, 30),
        new ShootBallsCommand(s_intake, s_shooter), new RotateToHeadingProfiledCommand(s_drive, 180),
        new DriveFowardCommand(s_drive, 0.4, 1));
    }
}