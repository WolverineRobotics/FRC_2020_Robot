package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverController;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimbCommand extends CommandBase {

    private DriverController dc;
    private ClimbSubsystem s_climb;

    public DefaultClimbCommand(ClimbSubsystem s_climb) {
        addRequirements(s_climb);
        this.s_climb = s_climb;
        dc = RobotContainer.getDriverController();
    }

    @Override
    public void execute() {
        double rightTrig = dc.getRightTrigger();
        double leftTrig = dc.getLeftTrigger();
        s_climb.setClimbSpeed((leftTrig - rightTrig)*0.75);

        if(dc.isPOVRight()){
            s_climb.setClimbLevelSpeed(-0.8);
        } else if(dc.isPOVLeft()){
            s_climb.setClimbLevelSpeed(0.8);
        } else {
            s_climb.setClimbLevelSpeed(0);
        }

        if (dc.isPressingB()){
            s_climb.setLock(false);
        } else if(dc.isPressingY()) {
            s_climb.setLock(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_climb.setClimbLevelSpeed(0);
        s_climb.setClimbSpeed(0);
    }

}