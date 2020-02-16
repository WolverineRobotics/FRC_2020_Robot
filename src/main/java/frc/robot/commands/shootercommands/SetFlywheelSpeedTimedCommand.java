package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetFlywheelSpeedTimedCommand extends CommandBase{

    private ShooterSubsystem s_shooter;
    private Timer timer;
    private double speed;
    private double totalTime;

    public SetFlywheelSpeedTimedCommand(ShooterSubsystem subsystem, double speed, double seconds){
        s_shooter = subsystem;
        addRequirements(subsystem);
        
        this.speed = speed;
        this.totalTime = seconds;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        s_shooter.setFlywheelSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return totalTime >= timer.get();
    }

}