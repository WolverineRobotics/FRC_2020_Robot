package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command will outake all of the balls it has through the bottom
 * of the robot (at entry location).
 * 
 * Command will finish once all of the sensors don't detect anything.
 */
public class OutakeBallsBottomCommand extends CommandBase {

    private IntakeSubsystem s_intake;

    public OutakeBallsBottomCommand(final IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        s_intake.setEntrySpeed(-RobotConst.IntakeConst.ENTRY_SPEED);
        s_intake.setCurveSpeed(-RobotConst.IntakeConst.CURVE_SPEED);
        s_intake.setVerticalLowerSpeed(-RobotConst.IntakeConst.LOWER_VERTICAL_SPEED);
        s_intake.setVerticalUpperSpeed(-RobotConst.IntakeConst.UPPER_VERTICAL_SPEED);
    }

    @Override
    public boolean isFinished() {
        boolean[] sen = s_intake.getSensors();
        if(!sen[0] && !sen[1] && !sen[2] && !sen[3] && !sen[4]) {
            return true;
        }
        return false;
    }

}