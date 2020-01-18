import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToVisionTargetCommand extends CommandBase{
    private final CameraSubsystem m_camerasubsystem;
    private final DriveSubsystem m_drivesubsystem;




    
    public RotateToVisionTargetCommand(CameraSubsystem camerasubsystem, DriveSubsystem driveSubsystem){
        m_camerasubsystem = cameraubsystem;
        m_drivesubsystem = drivesubsystem;
        addRequirements(camerasubsystem);
        addRequirements(drivesubsystem);  
    }
}