package frc.robot.commands.drivecommand;

import frc.robot.constants.RobotConst.PidConst;
import frc.robot.pid.LocationPID;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceLocationCommand extends DriveDistanceCommand {

    private double previousEncoderDistance;
    private LocationPID gyroPID;

    private final double DISTANCE_END = 0.2;

    public DriveDistanceLocationCommand(DriveSubsystem subsystem, double power, double distance, double heading,
            boolean brakeWhenFinished) {
        super(subsystem, power, distance, heading, brakeWhenFinished);
        this.power = power;
        this.gyroPID = new LocationPID(PidConst.GYRO_KP, PidConst.GYRO_KI, PidConst.GYRO_KD);

    }

    @Override
    public void initialize() {
        super.initialize();
        gyroPID.initialize(this.distance, this.heading);
        c_drive.resetEncoders();
        this.previousEncoderDistance = c_drive.getDistance();
    }

    @Override
    public boolean isFinished() {
        distance = gyroPID.getDistance();
        return (distance < DISTANCE_END);
        // return false;
    }

    @Override
    public void execute() {
        double leftSpeed, rightSpeed;
        double steering;

        double currentDistance = c_drive.getDistance();
        double distanceChanged = currentDistance - this.previousEncoderDistance;

        steering = gyroPID.calculate(c_drive.getPigeonHeading(), distanceChanged);

        // if (speed > Math.abs(power)) {
        //     speed = power;
        // }

        speed = this.power;

        leftSpeed = -(speed - steering);
        rightSpeed = speed + steering;

        c_drive.setSpeed(leftSpeed, rightSpeed);

        this.previousEncoderDistance = currentDistance;
    }

}
