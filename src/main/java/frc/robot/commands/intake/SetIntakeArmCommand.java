package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeArmCommand extends CommandBase { 

    private IntakeSubsystem s_intake;

    private boolean toOpen;

    public SetIntakeArmCommand(IntakeSubsystem s_intake, boolean toOpen) {
        this.s_intake = s_intake;
        this.toOpen = toOpen;

        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        s_intake.setIntakePiston(toOpen);
    }

    @Override
    public boolean isFinished() {
        boolean isDone = (toOpen && s_intake.isIntakeUp()) || (!toOpen && !s_intake.isIntakeUp());
        //done if command was set to open and intake is open or if the command was set to close and is closed
        return isDone;
    }
}