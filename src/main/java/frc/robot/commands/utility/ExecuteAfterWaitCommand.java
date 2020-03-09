package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ExecuteAfterWaitCommand extends SequentialCommandGroup {
    
    public ExecuteAfterWaitCommand(Command command, double timeSeconds) {
        addCommands(new WaitCommand(timeSeconds));
        addCommands(command);
    }

}
