package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends DriveDirectionCommand {
    double distance;

    boolean brakeWhenFinished;

    public DriveDistanceCommand(DriveSubsystem subsystem, double power, double distance, double heading,
            boolean brakeWhenFinished) {
        super(subsystem, power, heading);

        this.distance = distance;
        this.brakeWhenFinished = brakeWhenFinished;
    }

    @Override
    public void initialize() {
        super.initialize();

        c_drive.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Distance: " + c_drive.getDistance());
        if (this.power > 0) {
            return c_drive.getDistance() > distance;
        } else if (this.power < 0) {
            return c_drive.getDistance() < distance;
        }
        return true;

    }

    @Override
    public void end(boolean interupted) {
        super.end(interupted);
        System.out.println("Finished drive distance command at " + c_drive.getDistance());
        if (brakeWhenFinished) {
            c_drive.setForwardSpeed(0);
        }
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}