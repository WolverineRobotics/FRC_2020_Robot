package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.oi.OI;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem c_drive;   

    public DefaultDriveCommand (DriveSubsystem drive){
        c_drive = drive;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        c_drive.setForwardSpeed(0); 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Driver Left Stick Y and Driver Right Stick X
        double throttle = OI.getDriverThrottle();
        double turn = OI.getDriverTurn();

        double leftSpeed = 0;
        double rightSpeed = 0;

        leftSpeed = throttle - turn;
        rightSpeed = throttle + turn;

        if(OI.getFineControl()){
            //If fine control is active.
            c_drive.setSpeed(leftSpeed*0.3, -rightSpeed*0.3);
        } else {
            c_drive.setSpeed(leftSpeed*DriveConst.DRIVE_SPEED_REDUCTION_RATIO, -rightSpeed*DriveConst.DRIVE_SPEED_REDUCTION_RATIO);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

 }