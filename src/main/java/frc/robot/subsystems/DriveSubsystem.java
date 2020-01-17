package frc.robot.subsystems;

import static frc.robot.constants.RobotMap.DRIVE_LEFT_ENCODER_A;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_ENCODER_B;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_MOTOR_MASTER_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_MOTOR_SLAVE1_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_LEFT_MOTOR_SLAVE2_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_ENCODER_A;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_ENCODER_B;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_MOTOR_MASTER_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_MOTOR_SLAVE1_ADDRESS;
import static frc.robot.constants.RobotMap.DRIVE_RIGHT_MOTOR_SLAVE2_ADDRESS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.PidConst;
import frc.robot.constants.RobotMap;
import frc.robot.pid.GyroPID;

public class DriveSubsystem extends SubsystemBase {

    private Spark leftDrive01, rightDrive01;
    private VictorSPX leftDrive02, leftDrive03, rightDrive02, rightDrive03;
    private Encoder leftEncoder, rightEncoder;

    // private SpeedControllerGroup leftGroup, rightGroup;
    // private DifferentialDrive driveTrain;

    private AHRS navX;
    private PigeonIMU pigeon;

    public GyroPID gyroPID;

    public DriveSubsystem() {
        leftDrive01 = new Spark(DRIVE_LEFT_MOTOR_MASTER_ADDRESS);
        rightDrive01 = new Spark(DRIVE_RIGHT_MOTOR_MASTER_ADDRESS);

        leftDrive02 = new VictorSPX(DRIVE_LEFT_MOTOR_SLAVE1_ADDRESS);
        leftDrive03 = new VictorSPX(DRIVE_LEFT_MOTOR_SLAVE2_ADDRESS);
        rightDrive02 = new VictorSPX(DRIVE_RIGHT_MOTOR_SLAVE1_ADDRESS);
        rightDrive03 = new VictorSPX(DRIVE_RIGHT_MOTOR_SLAVE2_ADDRESS);

        // leftGroup = new SpeedControllerGroup(leftDrive01, leftDrive02, leftDrive03);
        // rightGroup = new SpeedControllerGroup(rightDrive01, rightDrive02,
        // rightDrive03);

        // driveTrain = new DifferentialDrive(leftGroup, rightGroup);

        leftEncoder = new Encoder(DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B);
        rightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B);

        leftEncoder.setDistancePerPulse(DriveConst.DRIVE_ENCODER_COUNTS_PER_INCH);
        rightEncoder.setDistancePerPulse(DriveConst.DRIVE_ENCODER_COUNTS_PER_INCH);

        navX = new AHRS(Port.kMXP);
        pigeon = new PigeonIMU(RobotMap.DRIVE_PIGEON_IMU_ADDRESS);

        gyroPID = new GyroPID(PidConst.GYRO_KP, PidConst.GYRO_KI, PidConst.GYRO_KD);
    }

    public void setLeftSpeed(double speed) {
        leftDrive01.set(speed);
        leftDrive02.set(ControlMode.PercentOutput, speed);
        leftDrive03.set(ControlMode.PercentOutput, speed);
    }

    public void setRightSpeed(double speed) {
        rightDrive01.set(speed);
        rightDrive02.set(ControlMode.PercentOutput, speed);
        rightDrive03.set(ControlMode.PercentOutput, speed);
    }

    public void setForwardSpeed(double speed) {
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
        return (leftDrive01.get() + leftDrive02.getMotorOutputPercent() + leftDrive03.getMotorOutputPercent()) / 3;
    }

    public double getRightSpeed() {
        return (rightDrive01.get() + rightDrive03.getMotorOutputPercent() + rightDrive03.getMotorOutputPercent()) / 3;
    }

    /**
     * Returns the gyro heading
     * 
     * @return double value in degrees (0-360 degrees)
     */
    public double getNavXHeading() {
        return (double) navX.getYaw();
    }

    /**
     * returns the gyro heading
     * 
     * @return double value in degrees
     */
    public double getPigeonHeading() {
        // double[] ypr = new double[3];
        // pigeon.getYawPitchRoll(ypr);
        // return ypr[0];
        return pigeon.getFusedHeading();
    }

    /**
     * resets the gyro heading
     */
    public void resetNavxHeading() {
        navX.reset();
    }

}