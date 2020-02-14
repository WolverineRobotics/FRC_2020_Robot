package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Will move all of the balls one stage upon intaking one ball.
 * 
 * Each stage is decided by the amount of balls in the magazine.
 * 
 * Stage 1: (One ball)
 * Ball 1 is at entry point activating Sensor one. 
 * To move to stage 2, the entry motor and curve motor must spin until sensor two is activated.
 * 
 * Stage 2: (Two Balls)
 * Ball 1 is at curve activating Sensor two
 * Ball 2 is at entry activating Sensor one
 * To move to stage 3, the entry, curve and vertical motors must spin until sensor three is activated.
 * 
 * Stage 3: (Three Balls)
 * Ball 1 is at first level of vertical conveyor activating Sensor three
 * Ball 2 is at curve activating Sensor two
 * Ball 3 is at entry activating Sensor one
 * To move to stage 4, the entry, curve, and veretical motors must spin until sensor four is activated.
 * 
 * Stage 4: (Four Balls)
 * Ball 1 is at second level of vertical conveyor activating Sensor four
 * Ball 2 is at first level of vertical conveyor activating Sensor three
 * Ball 3 is at curve activating Sensor two
 * Ball 4 is at entry activating Sensor one
 * 
 * Stage 5: (Five Balls)
 * Ball 1 is at third level of vertical conveyor activating Sensor five
 * Ball 2 is at second level of vertical conveyor activating Sensor four
 * Ball 3 is at first level of vertical conveyor activating Sensor three
 * Ball 4 is at curve activating Sensor two
 * Ball 5 is at entry activating Sensor one
 */
public class MoveBallsOneStageCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private int currentStage;

    private boolean isDone;

    public MoveBallsOneStageCommand(final IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
    }

    @Override
    public void initialize() {
        currentStage = s_intake.getAmountOfBalls();
        isDone = false;
    }

    @Override
    public void execute() {
        switch (currentStage) {
            case 0:
                isDone = true;
            case 1:
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                if(s_intake.isSensorTwoActivated()) {
                    isDone = true;
                }
                break;
            case 2:
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
                if(s_intake.isSensorThreeActivated()) {
                    isDone = true;
                }
                break;
            case 3:
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
                if(s_intake.isSensorFourActivated()) {
                    isDone = true;
                }
                break;
            case 4:
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
                if(s_intake.isSensorFiveActivated()) {
                    isDone = true;
                }
                break;
            case 5:
                isDone = true;
                break;
            default:
                //never supposed to reach here
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(s_intake.getAmountOfBalls() == 0 || isDone) {
            return true;
        }
        return false;
    }

}