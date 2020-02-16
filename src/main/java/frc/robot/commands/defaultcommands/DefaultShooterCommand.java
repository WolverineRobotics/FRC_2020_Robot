package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {

    private ShooterSubsystem s_shooter;

    public DefaultShooterCommand(ShooterSubsystem subsystem) {
        super();
        s_shooter = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}