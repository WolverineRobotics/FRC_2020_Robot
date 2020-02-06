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

    private double m_goal;

    public GyroToRotate(double trackWidthMeters, double goal) {
        this.trackWidthMeters = trackWidthMeters;
        m_goal = goal;
    }

    public void setGoal(double goal) {
        m_goal = goal;
    }

    public double getGoal() {
        return m_goal;
    }

    public double[] calculate(double currentGyroAngle) {
        // TODO: Normalize pigeon angle, get differance in angle
        double angleOff = Util.subtractGyroValues(m_goal, currentGyroAngle);

        // Distance = 2 * pi * r * angle / 360 deg
        double driveDistances = 2 * Math.PI * trackWidthMeters * angleOff / 360;

        // Reverse?
        double[] driveLeftRightDistance = { -driveDistances, driveDistances };
        return driveLeftRightDistance;
    }

}