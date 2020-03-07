package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private ShooterSubsystem s_shooter;

    
    private boolean isDone;

    private double flywheelSpeed;
    private int startMagSize;

    public ShootCommand(IntakeSubsystem s_intake, ShooterSubsystem s_shooter, double flywheelSpeed) {
        this.s_intake = s_intake;
        this.s_shooter = s_shooter;
        addRequirements(s_intake);
        addRequirements(s_shooter);

        isDone = false;

        this.flywheelSpeed = flywheelSpeed;
        startMagSize = s_intake.mag.size();

        if(flywheelSpeed < 0) {
            isDone = true;
        }

        if(startMagSize < 1) {
            isDone = true;
        }
    }

    @Override
    public void initialize() {
        s_shooter.setFlywheelSpeed(flywheelSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        s_shooter.setFlywheelSpeed(0);
    }

    @Override
    public boolean isFinished() {
        int currentMagSize = s_intake.mag.size();
        if((startMagSize - 1) == currentMagSize) { // a ball was shot
            isDone = true;
        }
        return isDone;
    }

}