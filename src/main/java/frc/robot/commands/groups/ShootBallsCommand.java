package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.MoveBallsToShootCommand;
import frc.robot.commands.shootercommands.SetFlywheelShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBallsCommand extends ParallelRaceGroup {

    private MoveBallsToShootCommand c_moveBallsToShoot;
    private SetFlywheelShootCommand c_setFlywheelShoot;

    public ShootBallsCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        c_moveBallsToShoot = new MoveBallsToShootCommand(intake);
        c_setFlywheelShoot = new SetFlywheelShootCommand(shooter);
        addCommands(c_moveBallsToShoot);
        addCommands(c_setFlywheelShoot);
    }

    @Override
    public void execute() {
        super.execute();

        // Passes if the flywheel is at speed from c_setFlywheelShoot to
        // c_moveBallsToShoot
        boolean flywheelAtSpeed = c_setFlywheelShoot.isFlywheelAtSpeed();
        c_moveBallsToShoot.setFlywheelReady(flywheelAtSpeed);
    }

}