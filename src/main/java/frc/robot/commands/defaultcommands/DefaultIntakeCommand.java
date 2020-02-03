package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private IntakeSubsystem s_intake;

    public DefaultIntakeCommand(IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        s_intake.setCurveSpeed(0);
        s_intake.setEntrySpeed(0);
        s_intake.setVerticalSpeed(0);
        s_intake.setShootSpeed(0);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    /**
     * Default commands never finish.
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}