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
        double rollersSpeed = 0;
        if(dc.isPOVLeft()) {
            rollersSpeed = 0.4;
        } else if(dc.isPOVRight()) {
            rollersSpeed = -0.4;
        }
        s_climb.setClimbLevelSpeed(rollersSpeed);

        double rightTrig = dc.getRightTrigger(); //climb up speed
        double leftTrig = dc.getLeftTrigger(); //climb down speed
        if(Math.abs(rightTrig) < 0.1) {
            rightTrig = 0;
        }
        if(Math.abs(leftTrig) < 0.1) {
            leftTrig = 0;
        }
        s_climb.setClimbSpeed(leftTrig - rightTrig);
    }

}