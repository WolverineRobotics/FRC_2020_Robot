/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.defaultcommands.DefaultCameraCommand;
import frc.robot.commands.defaultcommands.DefaultClimbCommand;
import frc.robot.commands.defaultcommands.DefaultDriveCommand;
import frc.robot.commands.defaultcommands.DefaultIntakeCommand;
import frc.robot.commands.defaultcommands.DefaultShooterCommand;
import frc.robot.commands.groups.AutonomousGroup;
import frc.robot.constants.RobotMap;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.oi.DriverController;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // all subsystem instances
  private DriveSubsystem s_drive;
  private IntakeSubsystem s_intake;
  private CameraSubsystem s_camera;
  private ShooterSubsystem s_shooter;
  private ClimbSubsystem s_climb;
  private ArduinoSubsystem s_arduino;

  // default commands
  private DefaultDriveCommand dc_drive;
  private DefaultIntakeCommand dc_intake;
  private DefaultCameraCommand dc_camera;
  private DefaultShooterCommand dc_shooter;
  private DefaultClimbCommand dc_climb;

  private Compressor compressor;

  // autonomous command
  private AutonomousGroup auto;

  // controllers
  private static DriverController control_driver;
  private static OperatorController control_operator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // CONTROLLERS
    control_driver = new DriverController(RobotMap.Controller.DRIVER);
    control_operator = new OperatorController(RobotMap.Controller.OPERATOR);

    // DRIVE
    s_drive = new DriveSubsystem();
    dc_drive = new DefaultDriveCommand(s_drive);
    CommandScheduler.getInstance().setDefaultCommand(s_drive, dc_drive);

    // INTAKE
    s_intake = new IntakeSubsystem();
    dc_intake = new DefaultIntakeCommand(s_intake);
    CommandScheduler.getInstance().setDefaultCommand(s_intake, dc_intake);

    // SHOOTER
    s_shooter = new ShooterSubsystem();
    dc_shooter = new DefaultShooterCommand(s_shooter);
    CommandScheduler.getInstance().setDefaultCommand(s_shooter, dc_shooter);

    // CAMERA
    s_camera = new CameraSubsystem();
    dc_camera = new DefaultCameraCommand(s_camera);
    CommandScheduler.getInstance().setDefaultCommand(s_camera, dc_camera);

    // CLIMB
    s_climb = new ClimbSubsystem();
    dc_climb = new DefaultClimbCommand(s_climb);
    CommandScheduler.getInstance().setDefaultCommand(s_climb, dc_climb);

    // ARDUINO
    s_arduino = new ArduinoSubsystem();

    compressor = new Compressor(3);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Placeholder command that finishes immediately
    return new CommandBase() {
      @Override
      public boolean isFinished() {
        return true;
      }
    };

  }

  public static DriverController getDriverController() {
    return control_driver;
  }

  public static OperatorController getOperatorController() {
    return control_operator;
  }
}
