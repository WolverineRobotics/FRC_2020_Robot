package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst.IntakeConst;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveBallsToShootCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private boolean flywheelReady = false;

    public MoveBallsToShootCommand(IntakeSubsystem subsystem) {
        super();
        s_intake = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (!s_intake.isSensorFiveActivated()) {
            // No ball in top position
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED,
                    IntakeConst.UPPER_VERTICAL_SPEED);
        } else {
            // Ball in top position,
            if (flywheelReady) {
                // Flywheel ready, shoot balls
                s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED,
                        IntakeConst.UPPER_VERTICAL_SHOOT_SPEED);
            } else {
                s_intake.setSpeeds(0, 0, 0, 0);
            }
        }
    }

    public void setFlywheelReady(boolean ready) {
        this.flywheelReady = ready;
    }

    @Override
    public boolean isFinished() {
        return (s_intake.getAmountOfBalls() == 0);
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.setSpeeds(0, 0, 0, 0);
    }
}