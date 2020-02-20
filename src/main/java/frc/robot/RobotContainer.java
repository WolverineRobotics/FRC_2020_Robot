/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.defaultcommands.DefaultDriveCommand;
import frc.robot.commands.defaultcommands.DefaultShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.defaultcommands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LidarSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // all subsystem instances
  private final DriveSubsystem s_drive;

  // default commands
  private final DefaultDriveCommand c_drive;

  // autonomous command

  private final LidarSubsystem s_lidar;

  private final ShooterSubsystem s_shooter;
  private final DefaultShooterCommand dc_shooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_drive = new DriveSubsystem();
    c_drive = new DefaultDriveCommand(s_drive);
    CommandScheduler.getInstance().setDefaultCommand(s_drive, c_drive);
    s_lidar = new LidarSubsystem();
    s_shooter = new ShooterSubsystem();
    dc_shooter = new DefaultShooterCommand(s_shooter);
    CommandScheduler.getInstance().setDefaultCommand(s_shooter, dc_shooter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Placeholder command that finishes immediately
    return new CommandBase(){
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
