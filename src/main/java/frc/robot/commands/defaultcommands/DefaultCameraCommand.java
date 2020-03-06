package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverController;
import frc.robot.subsystems.CameraSubsystem;

public class DefaultCameraCommand extends CommandBase {

    private CameraSubsystem c_camera;
    private DriverController dc;

    public DefaultCameraCommand(CameraSubsystem subsystem) {
        c_camera = subsystem;
        addRequirements(subsystem);
        dc = RobotContainer.getDriverController();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // c_camera.setLEDMode(CameraSubsystem.LEDMode.OFF);
        // c_camera.setCameraMode(CameraSubsystem.CameraMode.DRIVER);
        
        // Testing / debug
        c_camera.setLEDMode(CameraSubsystem.LEDMode.OFF);
        c_camera.setCameraMode(CameraSubsystem.CameraMode.VISION);
        c_camera.setSnapshotMode(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(dc.isLimelightLed()){
            c_camera.setLEDMode(CameraSubsystem.LEDMode.PIPELINE);

        } else {
            c_camera.setLEDMode(CameraSubsystem.LEDMode.OFF);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}
