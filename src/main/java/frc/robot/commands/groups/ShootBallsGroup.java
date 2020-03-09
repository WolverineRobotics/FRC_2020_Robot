package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.MoveBallsOneStageCommand;
import frc.robot.commands.intake.MoveBallsToShootCommand;
import frc.robot.commands.shootercommands.SetFlywheelShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBallsGroup extends ParallelRaceGroup {

    protected final MoveBallsToShootCommand c_moveBallsToShoot;
    protected final SetFlywheelShootCommand c_setFlywheelShoot;
    private boolean shootReady = true;

    public ShootBallsGroup(IntakeSubsystem intake, ShooterSubsystem shooter, double rpm) {
        this(intake, shooter);
        c_setFlywheelShoot.setSetpointRPM(rpm);
    }

    public ShootBallsGroup(IntakeSubsystem intake, ShooterSubsystem shooter) {
        c_moveBallsToShoot = new MoveBallsToShootCommand(intake);
        c_setFlywheelShoot = new SetFlywheelShootCommand(shooter);
        addCommands(c_moveBallsToShoot);
        addCommands(c_setFlywheelShoot);
    }

    public void setShootReady(boolean shoot) {
        this.shootReady = shoot;
    }

    @Override
    public void execute() {
        super.execute();

        // Passes if the flywheel is at speed from c_setFlywheelShoot to
        // c_moveBallsToShoot
        boolean flywheelAtSpeed = checkFlywheelAtSpeed();
        c_moveBallsToShoot.setFlywheelReady(flywheelAtSpeed && shootReady);
    }

    protected boolean checkFlywheelAtSpeed() {
        return c_setFlywheelShoot.isFlywheelAtSpeed();
    }

}