package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveFowardCommand;
import frc.robot.commands.intake.IntakeBallsUntilMagazine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveAndIntakeGroup extends ParallelRaceGroup{

    private final DriveSubsystem s_drive;
    private final IntakeSubsystem s_intake;

    public DriveAndIntakeGroup(DriveSubsystem drive, IntakeSubsystem intake, double speed, double distanceMeters){
        s_drive = drive;
        s_intake = intake;
        addCommands(
            new IntakeBallsUntilMagazine(s_intake, 3),
            new DriveFowardCommand(s_drive, speed, distanceMeters)
        );
    }

}
