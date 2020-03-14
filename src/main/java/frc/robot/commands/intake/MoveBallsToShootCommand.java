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
    }

    @Override
    public void execute() {
        if (!s_intake.isSensorFiveActivated()) {
            // No ball in top position
            s_intake.setSpeeds(0.1, 0.25, 0.3,
                    0.3);
            flywheelReady = false; 
            // System.out.println("Sensor 5 not activated, moving balls up");
        } else {
            // Ball in top position,
            if (flywheelReady) {
                // Flywheel ready, shoot balls
                s_intake.setSpeeds(0, 0, 0, 0.9);
                // System.out.println("Flywheel ready, shoot balls");
            } else {
                s_intake.setSpeeds(0, 0, 0, 0);
                // System.out.println("Flywheel not ready, holding");
            }
        }
        s_intake.updateSensorPositions();
    }

    public void setFlywheelReady(boolean ready) {
        this.flywheelReady = ready;
    }

    @Override
    public boolean isFinished() {
        // System.out.println("NUM OF BALLS: " + s_intake.getAmountOfBalls());
        return (s_intake.getAmountOfBalls() == 0);
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("MOVE BALLS TO SHOOT FINISHING");
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