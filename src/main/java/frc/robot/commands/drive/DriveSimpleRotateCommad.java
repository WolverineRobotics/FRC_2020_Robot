package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Util;

public class DriveSimpleRotateCommad extends CommandBase {
  private final DriveSubsystem m_drive;
  private double finalAngle;

  public DriveSimpleRotateCommad(DriveSubsystem subsystem, double angle) {
    m_drive = subsystem;
    addRequirements(subsystem);
    double pigeonAngle = m_drive.getPigeonHeading();
    finalAngle = pigeonAngle + angle;
    finalAngle = Util.normalizeValue(finalAngle, 0d, 360d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getGyroPID().setEnabled(true);
    m_drive.getGyroPID().setSetpoint(finalAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroCalculate = m_drive.getGyroPID().calculate(m_drive.getPigeonHeading());
    m_drive.setLeftSpeed(gyroCalculate);
    m_drive.setRightSpeed(-gyroCalculate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.getGyroPID().setEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = m_drive.getGyroPID().getError(m_drive.getPigeonHeading());
    if (error < 3) {
      return true;
    }
    return false;
  }

}