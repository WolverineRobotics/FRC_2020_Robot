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
        double rev = oc.getRightTrigger();
        if(Math.abs(rev) > 0.15) {
            s_shooter.setFlywheelSpeed(rev);
        // } else {
        //     s_shooter.setFlywheelSpeed(0);
        // }
        }else

        if(oc.isFlywheelSetVoltage()){
            s_shooter.setFlywheelVoltage(11);
        }
        else if(oc.isShooter10V()){
            s_shooter.setFlywheelVoltage(9.7);
        }
        else{
            s_shooter.setFlywheelSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}