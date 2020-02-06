package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveFowardCommand extends CommandBase {
    private final DriveSubsystem m_subsytem;
    private double speed;
    private double distance;

    public DriveFowardCommand(DriveSubsystem subsystem, double speed, double distance) {
        m_subsytem = subsystem;
        addRequirements(m_subsytem);
        this.speed = speed;
        this.distance = distance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsytem.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsytem.setForwardSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_subsytem.getDistance() > distance;

    }

}