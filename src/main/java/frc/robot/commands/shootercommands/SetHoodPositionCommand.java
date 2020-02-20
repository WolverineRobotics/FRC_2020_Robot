package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst.ShooterConst;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodPositionCommand extends CommandBase {

    private final int HOOD_ERROR_TOLERANCE = 1;

    private ShooterSubsystem s_shooter;
    private int hoodGoalPosition;
    private boolean finished = false;

    public SetHoodPositionCommand(ShooterSubsystem subsystem, int hoodPosition) {
        super();
        this.s_shooter = subsystem;
        this.hoodGoalPosition = hoodPosition;
    }

    @Override
    public void execute() {
        // If hood encoder not connected, log error and finish command
        if (!s_shooter.isHoodEncoderConnected()){
            System.err.println("Error! Hood Encoder not connected!");
            finished = true;
            return;
        }

        double error = s_shooter.getHoodError(hoodGoalPosition);
        double hoodSpeed;

        // TODO: May want to add proportional gain

        if(error > 0){
            hoodSpeed = (ShooterConst.HOOD_MOTOR_POWER);
        }else if(error < 0){
            hoodSpeed = (- ShooterConst.HOOD_MOTOR_POWER);
        }else{
            hoodSpeed = 0;
            finished = true;
        }

        s_shooter.setHoodSpeed(hoodSpeed);

    }

    @Override
    public boolean isFinished() {
        if(finished){
            return true;
        }

        // Checks if error is within tolerance
        double error =  s_shooter.getHoodError(hoodGoalPosition);
        return (Math.abs(error) <= HOOD_ERROR_TOLERANCE );
    }

    @Override
    public void end(boolean interrupted) {
        s_shooter.setHoodSpeed(0);
    }
}