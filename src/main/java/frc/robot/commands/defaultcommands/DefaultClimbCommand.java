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
        if(dc.isPOVUp()) {
            s_climb.setClimbSpeed(-1);
        } else if(dc.isPOVDown()) {
            s_climb.setClimbSpeed(1);
        } else {
            s_climb.setClimbSpeed(0);
        }
        double rightTrig = dc.getRightTrigger();
        double leftTrig = dc.getLeftTrigger();
        s_climb.setClimbLevelSpeed((rightTrig - leftTrig)*0.4);
    }

}