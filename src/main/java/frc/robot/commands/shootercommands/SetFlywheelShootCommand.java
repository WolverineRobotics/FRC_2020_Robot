package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Simple command to set the flywheel speed to shoot balls. Requires a command
 * group (or other class) to control when it's finished.
 */
public class SetFlywheelShootCommand extends CommandBase {

    private final double FLYWHEEL_SHOOT_RPM_TOLERANCE = 100;

    private final double VOLTAGE = 11;

    private final ShooterSubsystem s_shooter;

    private boolean finished = false;
    private double shotRPM = 4300;

    public SetFlywheelShootCommand(ShooterSubsystem shooter) {
        super();
        s_shooter = shooter;
        addRequirements(s_shooter);

    }

    public SetFlywheelShootCommand(ShooterSubsystem shooter, double rpm) {
        this(shooter);
        this.shotRPM = rpm;
    }

    @Override
    public void execute() {

        // System.out.println("SET FLYWHEEL SHOOT COMMAND 11 VOLTS");
        // s_shooter.setFlywheelRPM(FLYWHEEL_SHOOT_RPM, FLYWHEEL_BASE_POWER);
        s_shooter.setFlywheelVoltage(VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        // System.out.print("SET FLYWHEEL SHOOT FINISHED: ");
        // if(finished){
        // System.out.println("True");
        // }else{
        // System.out.println("False");
        // }
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
        // double error = s_shooter.getFlywheelSpeed() - FLYWHEEL_SHOOT_RPM;
        // return (Math.abs(error) <= FLYWHEEL_SHOOT_RPM_TOLERANCE);
        // System.out.print("FLYWHEEL AT SPEED? : ");
        // if(s_shooter.getFlywheelSpeed() >= shotRPM){
        // System.out.println("True");
        // }else{
        // System.out.println("False");
        // }
        return s_shooter.getFlywheelSpeed() >= shotRPM;
    }

    public double getCurrentRPM() {
        return s_shooter.getFlywheelSpeed();
    }

    public double getSetpointRPM() {
        return this.shotRPM;
    }

    public void setSetpointRPM(double rpm) {
        this.shotRPM = rpm;
    }
}