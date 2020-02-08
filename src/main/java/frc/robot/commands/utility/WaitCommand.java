package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

public class WaitCommand extends CommandBase {
    public Timer timer = new Timer();
    double time;
    public WaitCommand (double timeInSeconds, Subsystem... subsystems){
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double timerRightNow = timer.get();
    return timerRightNow > time;
  }
}