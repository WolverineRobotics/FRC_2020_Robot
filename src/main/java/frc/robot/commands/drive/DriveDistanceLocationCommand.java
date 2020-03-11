package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.pid.LocationPID;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceLocationCommand extends CommandBase {

    private final DriveSubsystem s_drive;
    private double previousEncoderDistance;
    private LocationPID gyroPID;
    private double heading;
    private double power;
    private double distance;

    private final double DISTANCE_END = 0.02;

    public DriveDistanceLocationCommand(DriveSubsystem drive, double speed, double heading, double distanceMeters) {
        // super(drive, speed * 11, heading, distanceMeters);
        // super(drive, heading);
        addRequirements(drive);
        this.s_drive = drive;
        this.gyroPID = new LocationPID(PIDConst.DRIVE_TURN_KP, PIDConst.DRIVE_TURN_KI, PIDConst.DRIVE_TURN_KD);
        this.heading = heading;
        this.distance = distanceMeters;
        this.power = speed;
    }

    @Override
    public void initialize() {
        super.initialize();
        gyroPID.initialize(this.distance, this.heading);
        s_drive.resetEncoders();
        this.previousEncoderDistance = s_drive.getDistance();
        s_drive.setDeadband(0);
    }

    @Override
    public boolean isFinished() {
        distance = gyroPID.getDistance();
        return (distance < DISTANCE_END);
        // return false;
    }

    @Override
    public void execute() {
        double steering;

        double currentDistance = s_drive.getDistance();
        double distanceChanged = currentDistance - this.previousEncoderDistance;

        steering = gyroPID.calculate(s_drive.getPigeonHeading(), distanceChanged);

        // if (speed > Math.abs(power)) {
        // speed = power;
        // }

        // System.out.println("Distance: " + currentDistance);
        // System.out.println("Power: " + power);
        // System.out.println("Steering: " + steering);

        s_drive.arcadeDrive(-power, steering, false);

        this.previousEncoderDistance = currentDistance;
    }

    @Override
    public void end(boolean interrupted) {
        s_drive.setForwardSpeed(0);
        s_drive.setDeadband(DriveConst.DRIVE_THORTTLE_TRIGGER_VALUE);
    }

}
