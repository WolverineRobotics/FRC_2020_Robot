package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst.IntakeConst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;
import frc.robot.subsystems.IntakeSubsystem.Ball;


public class MoveBallsToShootCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private boolean flywheelReady = false;

    public MoveBallsToShootCommand(IntakeSubsystem s_intake) {
        super();
        this.s_intake = s_intake;
        addRequirements(s_intake);
        s_intake.initAuto();
    }

    @Override
    public void execute() {
        if (!s_intake.isSensorFiveActivated()) {
            // No ball in top position
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED,
                    IntakeConst.UPPER_VERTICAL_SPEED);
            flywheelReady = false; 
        } else {
            // Ball in top position,
            if (flywheelReady) {
                // Flywheel ready, shoot balls
                s_intake.setSpeeds(0, 0, 0, 0.9);
            } else {
                s_intake.setSpeeds(0, 0, 0, 0);
            }
        }
        s_intake.updateSensorPositions();
    }

    public void setFlywheelReady(boolean ready) {
        this.flywheelReady = ready;
    }

    @Override
    public boolean isFinished() {
        System.out.println("NUM OF BALLS: " + s_intake.getAmountOfBalls());
        return (s_intake.getAmountOfBalls() == 0);
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.setSpeeds(0, 0, 0, 0);
    }

    private boolean isBallAtPositionFive() {
        for(Ball b : s_intake.mag) {
            if(b.getCurrentPosition() == Position.FIVE) {
                return true;
            }
        }
        return false;
    }
}