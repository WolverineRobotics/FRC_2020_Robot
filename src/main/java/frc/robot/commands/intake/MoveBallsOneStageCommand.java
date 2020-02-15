package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.RobotConst.IntakeConst;
import frc.robot.constants.RobotConst.IntakeConst.*;

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
        currentStage = 0;
        isDone = false;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        boolean[] sen = s_intake.getSensors();
        // evaluate the current stage and set the speeds to move balls to the next stage.
        // ! means ball detected
        // ? means no ball detected
        if(!sen[0] && !sen[1] && !sen[2] && !sen[3] && !sen[4]) { // ? ? ? ? ?
            // no stage, so there is no balls to move.
            currentStage = 0;
            s_intake.setSpeeds(0, 0, 0, 0);
            isDone = true;
        } else if(sen[0] && !sen[1] && !sen[2] && !sen[3] && !sen[4]) { // ! ? ? ? ? 
            currentStage = 1;
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, 0, 0);
        } else if(sen[0] && sen[1] && !sen[2] && !sen[3] && !sen[4]) {// ! ! ? ? ?
            currentStage = 2;
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED, 0);
        } else if(sen[0] && sen[1] && sen[2] && !sen[3] && !sen[4]) { // ! ! ! ? ? 
            currentStage = 3;
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED, 0);
        } else if(sen[0] && sen[1] && sen[2] && sen[3] && !sen[4]) { // ! ! ! ! ?
            currentStage = 4;
            s_intake.setSpeeds(IntakeConst.ENTRY_SPEED, IntakeConst.CURVE_SPEED, IntakeConst.LOWER_VERTICAL_SPEED, IntakeConst.UPPER_VERTICAL_SPEED);
        } else if(sen[0] && sen[1] && sen[2] && sen[3] && sen[4]) { // ! ! ! ! !
            // no stage to be moved.
            currentStage = 5;
            s_intake.setSpeeds(0, 0, 0, 0);
            isDone = true;
        } else { // stage is not identifiable
            s_intake.setSpeeds(0, 0, 0, 0);
            isDone = true;
        }
    }

    @Override
    public void execute() {
        boolean sen[] = s_intake.getSensors();
        switch(currentStage) {
            case 0: //should never reach here, but it's here for safety measures.
                isDone = true;
                return;
            case 1:
                if(sen[1] && !sen[2] && !sen[3] && !sen[4]) {
                    isDone = true;
                }
                break;
            case 2:
                if(sen[1] && sen[2] && !sen[3] && !sen[4]) {
                    isDone = true;
                }
                break;
            case 3:
                if(sen[1] && sen[2] && sen[3] && !sen[4]) {
                    isDone = true;
                }
                break;
            case 4:
                if(sen[1] && sen[2] && sen[3] && sen[4]) {
                    isDone = true;
                }
                break;
            case 5: //shoudl never reach here, but it's here for safety measures.
                isDone = true;
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.setSpeeds(0, 0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        if(s_intake.getAmountOfBalls() == 0 || isDone) {
            return true;
        }
        return false;
    }

}