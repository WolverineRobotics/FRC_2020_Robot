package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.pid.GyroPID;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToHeadingCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private GyroPID m_Gryo;
    private double heading;

    public RotateToHeadingCommand(DriveSubsystem subsystem, double heading) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_Gryo = m_drive.gyroPID;
        this.heading = heading;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Gryo.reset();
        m_Gryo.setSetpoint(heading);
        m_Gryo.setEnabled(true);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double steering = m_Gryo.calculate(m_drive.getPigeonHeading());
        if (steering >= 0.3) {
            steering = 0.3;
        } else if (steering <= 0.3) {
            steering = -0.3;
        }
        m_drive.setLeftSpeed(steering);
        m_drive.setRightSpeed(-steering);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Gryo.setEnabled(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_Gryo.getError()) < 2);

    }

}