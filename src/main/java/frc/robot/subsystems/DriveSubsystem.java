package frc.robot.subsystems;

import static frc.robot.constants.RobotMap.DRIVE_LEFT_ENCODER_A;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_ENCODER_B;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_MOTOR_MASTER_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_MOTOR_SLAVE_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_ENCODER_A;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_ENCODER_B;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_MOTOR_MASTER_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst.PidConst;
import frc.robot.constants.RobotMap;
import frc.robot.pid.GyroPID;

public class DriveSubsystem extends SubsystemBase {

    private Spark leftDrive01, leftDrive02, rightDrive01, rightDrive02;
    private Encoder leftEncoder, rightEncoder;

    private AHRS navX;
    private PigeonIMU pigeon;

    public GyroPID gyroPID;

    public DriveSubsystem() {
        leftDrive01 = new Spark(DRIVE_LEFT_MOTOR_MASTER_ADDRESS);
        leftDrive02 = new Spark(DRIVE_LEFT_MOTOR_SLAVE_ADDRESS);
        rightDrive01 = new Spark(DRIVE_RIGHT_MOTOR_MASTER_ADDRESS);
        rightDrive02 = new Spark(DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS);

        leftEncoder = new Encoder(DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B);
        rightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B);

        navX = new AHRS(Port.kMXP);
        pigeon = new PigeonIMU(RobotMap.DRIVE_PIGEON_IMU_ADDRESS);

        gyroPID = new GyroPID(PidConst.GYRO_KP, PidConst.GYRO_KI, PidConst.GYRO_KD);
    }

    public void setLeftSpeed(double speed) {
        leftDrive01.set(speed);
        leftDrive02.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightDrive01.set(speed);
        rightDrive02.set(speed);
    }

    public void setFwdSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public double getDistanceLeftEncoder() {
        return leftEncoder.getDistance();
    }

    public double getDistanceRightEncoder() {
        return rightEncoder.getDistance();
    }

    public double getDistance() {
        return (getDistanceLeftEncoder() + getDistanceRightEncoder()) / 2;
    }

    /**
     * resets encoder values
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * returns the absolute raw encoder counts of the left encoder
     * 
     * @return int number of counts to deliver
     */
    public int getRawLeftEncoder() {
        return leftEncoder.get();
    }

    /**
     * returns the absolute encoder counts of the right encoder
     * 
     * @return int number of counts to deliver
     */
    public int getRawRightEncoder() {
        return rightEncoder.get();
    }

    public double getLeftSpeed() {
        return (leftDrive01.get() + leftDrive02.get()) / 2;
    }

    public double getRightSpeed() {
        return (rightDrive01.get() + rightDrive02.get()) / 2;
    }

    /**
     * returns the gyro heading
     * 
     * @return double value in degrees
     */
    public double getNavxHeading() {
        return (double) navX.getYaw();
    }

    /**
     * returns the gyro heading
     * 
     * @return double value in degrees
     */
    public double getPigeonHeading() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    /**
     * resets the gyro heading
     */
    public void resetNavxHeading() {
        navX.reset();
    }

}