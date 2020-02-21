package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConst;
import frc.robot.oi.OperatorController;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {

    private ShooterSubsystem s_shooter;
    private OperatorController oc;

    public DefaultShooterCommand(ShooterSubsystem subsystem) {
        s_shooter = subsystem;
        addRequirements(subsystem);
        oc = RobotContainer.getOperatorController();
    }

    @Override
    public void execute() {
        if(oc.isFlyWheelRun()) {
            s_shooter.setFlywheelSpeed(RobotConst.ShooterConst.SHOOTER_SPEED);
        } else {
            s_shooter.setFlywheelSpeed(0);
        }
        if(oc.getHoodRotation() > 0.3 || oc.getHoodRotation() < -0.3) { //deadzone is 0.3
            s_shooter.setHoodSpeed(oc.getHoodRotation()*0.2);
        } else {
            s_shooter.setHoodSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}