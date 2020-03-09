package frc.robot.commands.drive;

import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.pid.LocationPID;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceLocationCommand extends DriveDirectionCommand {

    private final DriveSubsystem s_drive;
    private double previousEncoderDistance;
    private LocationPID gyroPID;
    private double heading;
    private double power;

    private final double DISTANCE_END = 0.02;

    public DriveDistanceLocationCommand(DriveSubsystem drive, double speed, double heading, double distanceMeters) {
        super(drive, speed *11, heading, distanceMeters);
        this.s_drive = drive;
        this.gyroPID = new LocationPID(PIDConst.GYRO_KP, PIDConst.GYRO_KI, PIDConst.GYRO_KD);
        this.heading = heading;
    }

    @Override
    public void initialize() {
        super.initialize();
        gyroPID.initialize(this.distance, this.heading);
        s_drive.resetEncoders();
        this.previousEncoderDistance = s_drive.getDistance();
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

        s_drive.arcadeDrive(power, steering, false);

        this.previousEncoderDistance = currentDistance;
    }

}
