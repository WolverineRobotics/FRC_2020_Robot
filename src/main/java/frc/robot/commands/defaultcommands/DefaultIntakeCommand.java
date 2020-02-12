package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.groups.ShootGroup;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private OperatorController oc;

    public DefaultIntakeCommand(IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
        oc = RobotContainer.getOperatorController();
    }

    @Override
    public void initialize() {
        s_intake.setCurveSpeed(0);
        s_intake.setEntrySpeed(0);
        s_intake.setVerticalSpeed(0);
        s_intake.setShootSpeed(0);
    }

    @Override
    public void execute() {
        if (oc.isHoldingLeftTrigger()) { // if is intaking
            int amountOfBalls = s_intake.getAmountOfBalls();
            if (amountOfBalls == 1) { // 1 ball in currently
                s_intake.setEntrySpeed(0.5);
            } else if (amountOfBalls == 2) { // 2 balls in currnetly
                s_intake.setEntrySpeed(0.5);
                s_intake.setCurveSpeed(0.5);
            } else if (amountOfBalls == 3) { // 3 balls in currently
                s_intake.setEntrySpeed(0.5);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
            } else if (amountOfBalls == 4) { // 4 balls in currently
                s_intake.setEntrySpeed(0.5);
                s_intake.setCurveSpeed(0.5);
                s_intake.setVerticalSpeed(0.5);
            } else if (amountOfBalls == 5) { // 5 balls in currently
                // do nothing
            } else { // no balls in currently
                s_intake.setEntrySpeed(0.5);
            }
        } else if(oc.isHoldingRightTrigger()) { //if operator outaking
            s_intake.setShootSpeed(1);
            //conveyors will move at a slower speed so that shoot can ramp up
            s_intake.setEntrySpeed(0.1);
            s_intake.setCurveSpeed(0.1);
            s_intake.setVerticalSpeed(0.1);
        } else if(oc.getAutoShootButton()) { // if operator wants to auto shoot
            new SequentialCommandGroup(
                
            );
        }
    }

    @Override
    public void end(boolean interrupted) {

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