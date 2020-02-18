package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst;
import frc.robot.oi.DriverController;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimbCommand extends CommandBase{
    
    private DriverController c_driver; 
    private ClimbSubsystem c_climb;
    private Double motorSpeed;

    public DefaultClimbCommand(ClimbSubsystem subsystem, double Speed){
        c_climb = subsystem;
        addRequirements(subsystem);
        motorSpeed = Speed;
    }
    
    @Override
    public void initialize(){
        c_climb.setClimbLevel(0);
        c_climb.setClimbLock(false);
        c_climb.setClimbSpeed(0);
    }

    @Override
    public void execute(){
        if (c_climb.getClimbLock() != DoubleSolenoid.Value.kForward){
            if (c_driver.isClimbDown()){
                if (c_climb.getSensor()){
                    c_climb.setClimbLock(true);
                } else {
                    c_climb.setClimbSpeed(-motorSpeed); //TODO: double check polarity
                }
            } else if (c_driver.isClimbUp()){
                c_climb.setClimbSpeed(motorSpeed);
            }
        }
        // TODO: Check polarity 
        if (c_driver.isRight()){
            c_climb.setClimbSpeed(RobotConst.ControllerConst.CLIMB_LEVEL_SPEED);
        } else if (c_driver.isLeft()) {
            c_climb.setClimbLevel(-RobotConst.ControllerConst.CLIMB_LEVEL_SPEED);
        }
    }
}