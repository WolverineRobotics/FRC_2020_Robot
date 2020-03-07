package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;
import frc.robot.subsystems.IntakeSubsystem.Ball;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveBallsOneStageCommand extends CommandBase {

    private IntakeSubsystem s_intake;

    public MoveBallsOneStageCommand(IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(this.s_intake);
    }

    @Override
    public void initialize() {
        s_intake.moveBallsOneStage();
    }

    @Override
    public void execute() {

        s_intake.setMoveBalls(true);
    }

    @Override
    public boolean isFinished() {
        List<Ball> mag = s_intake.mag;
        for(Ball b : mag) {
            if(!b.isAtDestination()) {
                return false;
            }
        }
        return true;
    }

}