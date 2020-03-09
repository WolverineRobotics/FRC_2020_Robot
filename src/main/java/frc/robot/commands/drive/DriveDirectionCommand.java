package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveVoltage;

public class DriveDirectionCommand extends RotateToHeadingProfiledCommand {

    private double driveVoltage;
    protected double distance;
    private final double startingDistance;
    private double maxVoltage;

    public DriveDirectionCommand(DriveSubsystem drive, double voltage, double heading, double distanceMeters) {
        super(drive, heading);
        driveVoltage = voltage;
        startingDistance = s_drive.getDistance();

        maxTurnVoltage = 6;
    }


    @Override
    protected void setVoltages(DriveVoltage turnVoltage) {
        System.out.println("Left Turn Voltage 1: " + turnVoltage.leftVoltage);
        System.out.println("Right Turn Voltage 1: " + turnVoltage.rightVoltage);
        turnVoltage = DriveVoltage.clampVoltage(turnVoltage, maxVoltage);
        DriveVoltage forwardVoltage = new DriveVoltage(driveVoltage, driveVoltage);
        DriveVoltage totalVoltage = DriveVoltage.addVoltage(forwardVoltage, turnVoltage);
        totalVoltage = DriveVoltage.clampVoltage(totalVoltage, maxVoltage);
        System.out.println("Left Voltage 1: " + totalVoltage.leftVoltage);
        System.out.println("Right Voltage 1: " + totalVoltage.rightVoltage);
        s_drive.setVoltage(totalVoltage.leftVoltage, totalVoltage.rightVoltage);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() && s_drive.getDistance() >= distance + startingDistance;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    public void setDriveVoltage(double voltage) {
        this.driveVoltage = voltage;
    }
}