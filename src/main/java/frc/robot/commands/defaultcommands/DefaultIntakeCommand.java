package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.RotateToVisionTargetCommand;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private final IntakeSubsystem s_intake;
    private final OperatorController oc;

    public DefaultIntakeCommand(final IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
        oc = RobotContainer.getOperatorController();
    }

    @Override
    public void initialize() {
        s_intake.setCurveSpeed(0);
        s_intake.setEntrySpeed(0);
        s_intake.setVerticalSpeed(0);
    }

    @Override
    public void execute() {
        if (oc.isHoldingLeftTrigger()) { // if is intaking
            if(s_intake.isIntakeOpen()) {
                s_intake.setIntakePiston(true);
            }
            final int amountOfBalls = s_intake.getAmountOfBalls();
            final boolean[] sen = s_intake.getSensors();
            if(!sen[0]) { //if sensor 1 is not detecting anything and there are 4+ balls in the mag
                s_intake.setEntrySpeed(0.5);
            } else if(sen[0]) {
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
            } else if(sen[0] && sen[1]) {
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
            } else if(sen[0] && sen[1] && sen[2] && sen[3] && !sen[4]) {
                s_intake.setEntrySpeed(0.3);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
            } else if(sen[0] && sen[1] && sen[2] && !sen[3] && !sen[4]) {

            } else {
                s_intake.setEntrySpeed(0.5);
            }
        } else if(oc.isHoldingRightTrigger()) { //if operator outaking
            s_intake.setEntrySpeed(0.1);
            s_intake.setCurveSpeed(0.1);
            s_intake.setVerticalSpeed(0.1);
        } else if(oc.getAutoShootButton()) { // if operator wants to auto shoot
            /**
            * Operator presses one button and the robot will:
            * 1. Rotate to vision target, will cancel command if none found
            * 2. Spin the fly wheel for X seconds
            * 3. Run all of the conveyors to unload magazine.
            */
            new SequentialCommandGroup(
                // new RotateToVisionTargetCommand(s_camera, s_drive);
                
            );
        }
    }

    /**
     * Default commands never finish.
     * 
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}