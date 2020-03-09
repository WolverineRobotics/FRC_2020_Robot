package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallsUntilMagazine extends CommandBase {

    private IntakeSubsystem s_intake;

    private int desiredMagAmount;
    private boolean isDone;
    
    /**
     * Will not run command if Intake is Up
     * @param s_intake Intake Subsystem instance
     * @param desiredMagAmount Will run the command until the magazine has this number of balls
     */
    public IntakeBallsUntilMagazine(IntakeSubsystem s_intake, int desiredMagAmount) {
        this.desiredMagAmount = desiredMagAmount;
        this.s_intake = s_intake;
        addRequirements(s_intake);
        isDone = false;

        if(s_intake.isIntakeUp()) {
            isDone = true;
        }
    }

    @Override
    public void execute() {
        s_intake.setMoveBalls(true);
    }

    @Override
    public boolean isFinished() {
        int currentMagAmount = s_intake.mag.size();
        if(currentMagAmount >= desiredMagAmount) {
            isDone = true;
        }
        return isDone;
    }

}
