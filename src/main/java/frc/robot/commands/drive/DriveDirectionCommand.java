package frc.robot.commands.drivecommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.JoystickMap;
import frc.robot.oi.OI;
import frc.robot.pid.GyroPID;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDirectionCommand extends CommandBase {
    protected double power, heading, speed;

    protected GyroPID gyroPID;

    protected DriveSubsystem c_drive;

    public DriveDirectionCommand(DriveSubsystem subsystem, double power, double heading) {
        c_drive = subsystem;
        System.out.println("Requires Drivesubsytem " + c_drive);
        addRequirements(c_drive);

        gyroPID = c_drive.gyroPID;

        this.power = power;
        this.heading = heading;
        this.speed = 0;
    }

    @Override
    public void initialize() {
        gyroPID.setSetpoint(heading);
        gyroPID.setEnabled(true);
    }

    @Override
    public void execute() {
        double leftSpeed, rightSpeed;
        double steering;

        steering = gyroPID.calculate(c_drive.getPigeonHeading());

        // if(speed > Math.abs(power)){
        // speed = power;
        // }

        speed = power;

        leftSpeed = -(speed - steering);
        rightSpeed = speed + steering;

        c_drive.setSpeed(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return OI.getDriver().getRawButton(JoystickMap.BUTTON_SELECT);
    }

    @Override
    public void end(boolean interrupted) {
        c_drive.setForwardSpeed(0);
        gyroPID.setEnabled(false);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setHeading(double heading) {
        c_drive.gyroPID.setSetpoint(heading);
        this.heading = heading;
    }
}