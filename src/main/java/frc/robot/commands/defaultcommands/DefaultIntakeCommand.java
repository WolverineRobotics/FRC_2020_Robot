package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.RobotConst.IntakeConst;

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
        s_intake.setEntrySpeed(0);
        s_intake.setCurveSpeed(0);
        s_intake.setVerticalLowerSpeed(0);
        s_intake.setVerticalUpperSpeed(0);
    }

    @Override
    public void execute() {
        if(oc.isHoldingRightTrigger()) { // if operator wants to dumby proof intake
            s_intake.setMoveBalls(true);
        } else if(oc.isResetIntakeLogic()) { // if operator wants to auto shoot
            /**
            * Operator presses one button and the robot will:
            * 1. Rotate to vision target, will cancel command if none found
            * 2. Spin the fly wheel for X seconds
            * 3. Run all of the conveyors to unload magazine.
            */
            s_intake.mag.clear();
            s_intake.unfinishedDesto.clear();
            s_intake.ballsToRemove.clear();
            s_intake.currentPossessions.clear();
        } else if(oc.isOutaking()) { //if operator wants to outake the balls from the bottom
            s_intake.setEntrySpeed(-0.3);
            s_intake.setCurveSpeed(-0.5);
            s_intake.setVerticalLowerSpeed(-0.5);
            s_intake.setVerticalUpperSpeed(-0.5);
        } else if(oc.isPOVDown()) { //if operator wants to move balls all to fly wheel
            s_intake.setEntrySpeed(0.1);
            s_intake.setCurveSpeed(0.3);
            s_intake.setVerticalLowerSpeed(0.2);
            s_intake.setVerticalUpperSpeed(0.5);
        } else if(oc.isPressingB()) { //move front piston
            if(s_intake.isIntakeOpen()) {
                s_intake.setIntakePiston(false);
            } else {
                s_intake.setIntakePiston(true);
            }
        } else {
            s_intake.setMoveBalls(false);
            s_intake.setSpeeds(0, 0, 0, 0);
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