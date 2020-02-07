package frc.robot.pid;

import frc.robot.util.Util;

/**
 * Calculates the distances, in meters, that each set of wheels of the
 * drivetrain need to move to rotate to a certain gyro angle. It is not meant to
 * be 100% precise and should be recalculated every iteration of the main
 * control loop.
 */
public class GyroToRotate {

    private final double trackWidthMeters;

    public GyroToRotate(double trackWidthMeters) {
        this.trackWidthMeters = trackWidthMeters;
    }

    public double[] calculate(double currentGyroAngle, double goal) {
        // Normalize pigeon angle, get differance in angle
        double angleOff = Util.subtractGyroValues(goal, currentGyroAngle);

        // Distance = 2 * pi * r * angle / 360 deg
        double driveDistances = 2 * Math.PI * trackWidthMeters * angleOff / 360;

        // Reverse?
        double[] driveLeftRightDistance = { -driveDistances, driveDistances };
        return driveLeftRightDistance;
    }

}