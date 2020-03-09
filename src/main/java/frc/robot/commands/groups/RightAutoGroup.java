package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveFowardCommand;
import frc.robot.commands.drive.RotateToHeadingProfiledCommand;
import frc.robot.commands.drive.RotateToVisionTargetCommand;
import frc.robot.commands.intake.SetIntakeArmCommand;
import frc.robot.commands.shootercommands.SetFlywheelSpeedTimedCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RightAutoGroup extends SequentialCommandGroup{

    private final DriveSubsystem s_drive;
    private final IntakeSubsystem s_intake;
    private final ShooterSubsystem s_shooter;
    private final CameraSubsystem s_camera;

    public RightAutoGroup(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, CameraSubsystem camera){
        this.s_drive = drive;
        this.s_intake = intake;
        this.s_shooter = shooter;
        this.s_camera = camera;
        addCommands(
            new RotateToHeadingProfiledCommand(s_drive, 30),

            new AlignAndShootGroup(s_drive, s_intake, s_shooter, s_camera), 
            new SetIntakeArmCommand(s_intake, false),

            new RotateToHeadingProfiledCommand(s_drive, 180-1),
            new DriveAndIntakeGroup(s_drive, s_intake, 0.4, Units.inchesToMeters(192.75)),
            new RotateToHeadingProfiledCommand(s_drive, 10),

            // new DriveFowardCommand(s_drive, 0.7, Units.inchesToMeters(178)),

            new ParallelRaceGroup(new DriveFowardCommand(s_drive, 0.7, Units.inchesToMeters(178)),
            new SetFlywheelSpeedTimedCommand(s_shooter, 0.85, 9999)),
            

            new AlignAndRapidFireGroup(s_drive, s_intake, s_shooter, s_camera, 4300)
        );

    }

    private void reset(){

    }

}