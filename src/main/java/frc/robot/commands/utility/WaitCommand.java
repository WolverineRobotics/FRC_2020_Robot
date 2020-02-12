package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

public class WaitCommand extends CommandBase {
    private Timer timer;
    private double time;

    public WaitCommand (double timeInSeconds, Subsystem... subsystems){
      timer = new Timer();
      for(Subsystem sub: subsystems){
        addRequirements(sub);
      }
      time = timeInSeconds;
    }
    
  @Override
  public void initialize() {
    timer.start(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double timerRightNow = timer.get();
    return timerRightNow >= time;
  }
} 