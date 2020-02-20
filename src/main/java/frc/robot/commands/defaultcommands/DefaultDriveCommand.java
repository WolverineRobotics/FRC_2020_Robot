package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem c_drive;

    public DefaultDriveCommand(DriveSubsystem drive) {
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
        double throttle = RobotContainer.getDriverController().getThrottle();
        double turn = RobotContainer.getDriverController().getTurn();

        if (RobotContainer.getDriverController().getFineControl()) {
            // If fine control is active.
            arcadeDrive(throttle, turn, DriveConst.DRIVE_SPEED_REDUCTION_RATIO_FINE);
        } else {
            arcadeDrive(throttle, turn, DriveConst.DRIVE_SPEED_REDUCTION_RATIO);
        }

    }

    private void arcadeDrive(double throttle, double turn, double speedReduction) {
        arcadeDrive(throttle, turn, speedReduction, false);
    }

    private void arcadeDrive(double throttle, double turn, double speedReduction, boolean invertTurn) {
        if (invertTurn) {
            turn = -turn;
        }
        throttle *= speedReduction;
        turn *= speedReduction;
        c_drive.arcadeDrive(throttle, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}