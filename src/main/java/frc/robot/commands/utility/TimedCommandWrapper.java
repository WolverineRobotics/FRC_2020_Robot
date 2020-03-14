package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedCommandWrapper extends ParallelRaceGroup {
    public TimedCommandWrapper(Command command, double timeoutSeconds){
        addCommands(command);
        addCommands(new WaitCommand(timeoutSeconds));
    }
}
