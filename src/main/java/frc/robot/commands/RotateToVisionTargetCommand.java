package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivecommand.RotateToHeadingCommand;
import frc.robot.exceptions.NtEntryNullException;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CameraSubsystem.LimelightLEDMode;
import frc.robot.subsystems.CameraSubsystem.LimelightVisionMode;

public class RotateToVisionTargetCommand extends CommandBase{
    private final CameraSubsystem m_camera;
    private final DriveSubsystem m_drive;
    double counter = 0;
    
    public RotateToVisionTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem){
        m_camera = cameraSubsystem;
        m_drive = driveSubsystem;
        addRequirements(cameraSubsystem);
        addRequirements(driveSubsystem);     
    }
    @Override
    public void initialize() {
        m_camera.setLedMode(LimelightLEDMode.ON); 
        m_camera.setCamMode(LimelightVisionMode.VISION);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        try{
            double xDegOff = m_camera.getXDegOff();
            counter  = 2301;
            RotateToHeadingCommand(xDegOff);
        } catch (NtEntryNullException exce){
            exce.printStackTrace();
            counter += 1; 
        }
        

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return counter > 2300;
    }
}