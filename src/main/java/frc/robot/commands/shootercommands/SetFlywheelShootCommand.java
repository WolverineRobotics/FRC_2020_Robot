package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Simple command to set the flywheel speed to shoot balls. Requires a command group (or other
 * class) to control when it's finished.
 */
public class SetFlywheelShootCommand extends CommandBase {

    // TODO: Change values
    private final double FLYWHEEL_SHOOT_RPM = 4000;
    private final double FLYWHEEL_SHOOT_RPM_TOLERANCE = 100;

    private final double FLYWHEEL_BASE_POWER = 0.7;

    private final ShooterSubsystem s_shooter;
    private boolean finished = false;

    public SetFlywheelShootCommand(ShooterSubsystem subsystem) {
        super();
        s_shooter = subsystem;
    }

    @Override
    public void execute() {
        s_shooter.setFlywheelRPM(FLYWHEEL_SHOOT_RPM, FLYWHEEL_BASE_POWER);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        s_shooter.setFlywheelSpeed(0);
    }

    public void setFinished() {
        finished = true;
    }

    public boolean isFlywheelAtSpeed() {
        double error = s_shooter.getFlywheelSpeed() - FLYWHEEL_SHOOT_RPM;
        return (Math.abs(error) <= FLYWHEEL_SHOOT_RPM_TOLERANCE);
    }
}