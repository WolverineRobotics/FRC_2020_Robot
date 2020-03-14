package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceLocationCommand;
import frc.robot.commands.drive.DriveFowardCommand;
import frc.robot.commands.drive.RotateToHeadingProfiledCommand;
import frc.robot.commands.drive.RotateToVisionTargetCommand;
import frc.robot.commands.intake.SetIntakeArmCommand;
import frc.robot.commands.shootercommands.SetFlywheelSpeedTimedCommand;
import frc.robot.commands.utility.TimedCommandWrapper;
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

            new SetIntakeArmCommand(s_intake, false),
            new TimedCommandWrapper(new AlignAndShootGroup(s_drive, s_intake, s_shooter, s_camera, 4200), 5),
            

            // new RotateToHeadingProfiledCommand(s_drive, 180),
            new DriveLocationAndIntakeGroup(s_drive, s_intake, 0.35, 180, Units.inchesToMeters(192.75)),

            // new RotateToHeadingProfiledCommand(s_drive, 10),

            // new DriveFowardCommand(s_drive, 0.8, Units.inchesToMeters(178)),

            new ParallelRaceGroup(new DriveDistanceLocationCommand(s_drive, 0.85, 10, Units.inchesToMeters(178)),
            new SetFlywheelSpeedTimedCommand(s_shooter, 0.85, 9999)),
            // new ParallelRaceGroup(new DriveDistanceLocationCommand(s_drive, 0.4, 10, Units.inchesToMeters(24)),
            // new SetFlywheelSpeedTimedCommand(shooter, 0.85, 999){
            //     @Override
            //     public boolean isFinished() {
            //         return false;
            //     }
            // }
            // ),

            new AlignAndShootGroup(s_drive, s_intake, s_shooter, s_camera, 4200)
        );

    }

    private void reset(){

    }

}