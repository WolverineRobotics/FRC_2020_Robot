package frc.robot.subsystems;


import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.ControllerConst;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.Drive;
import frc.robot.pid.DriveFeedForwardPID;
import frc.robot.pid.GyroPID;
import frc.robot.pid.GyroToRotate;

public class DriveSubsystem extends SubsystemBase {

    private Spark leftDrive01, leftDrive02, rightDrive01, rightDrive02;
    private Encoder leftEncoder, rightEncoder;

    private SpeedControllerGroup leftGroup, rightGroup;
    private DifferentialDrive driveTrain;

    private DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveOdometry m_odometry;

    private AHRS navX;
    private PigeonIMU pigeon;

    public DriveFeedForwardPID leftPid;
    public DriveFeedForwardPID rightPid;

    private GyroToRotate gyroToRotate;

    private GyroPID gyroPID;

    public DriveSubsystem() {
        super();

        leftDrive01 = new Spark(RobotMap.Drive.DRIVE_LEFT_MOTOR_MASTER_ADDRESS);
        leftDrive02 = new Spark(RobotMap.Drive.DRIVE_LEFT_MOTOR_SLAVE_ADDRESS);
        rightDrive01 = new Spark(RobotMap.Drive.DRIVE_RIGHT_MOTOR_MASTER_ADDRESS);
        rightDrive02 = new Spark(RobotMap.Drive.DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS);

        leftGroup = new SpeedControllerGroup(leftDrive01, leftDrive02);
        rightGroup = new SpeedControllerGroup(rightDrive01, rightDrive02);
        rightGroup.setInverted(true);

        driveTrain = new DifferentialDrive(leftGroup, rightGroup);

        leftEncoder = new Encoder(RobotMap.Drive.DRIVE_LEFT_ENCODER_A, RobotMap.Drive.DRIVE_LEFT_ENCODER_B);
        rightEncoder = new Encoder(RobotMap.Drive.DRIVE_RIGHT_ENCODER_A, RobotMap.Drive.DRIVE_RIGHT_ENCODER_B);

        leftEncoder.setDistancePerPulse(DriveConst.DRIVE_ENCODER_COUNTS_PER_INCH);
        rightEncoder.setDistancePerPulse(DriveConst.DRIVE_ENCODER_COUNTS_PER_INCH);

        navX = new AHRS(Port.kMXP);
        pigeon = new PigeonIMU(Drive.DRIVE_PIGEON_IMU_ADDRESS);

        gyroPID = new GyroPID(PIDConst.GYRO_KP, PIDConst.GYRO_KI, PIDConst.GYRO_KD);

        m_kinematics = new DifferentialDriveKinematics(CharacterizationConst.K_TRACKWIDTH_METERS);
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getPigeonHeading()));

        leftPid = new DriveFeedForwardPID();
        rightPid = new DriveFeedForwardPID();

        gyroToRotate = new GyroToRotate(CharacterizationConst.K_TRACKWIDTH_METERS);

        setDeadband(ControllerConst.DRIVE_THORTTLE_TRIGGER_VALUE);

        SendableRegistry.addLW(leftPid, "[Drive] Left PID");
        SendableRegistry.addLW(rightPid, "[Drive] Right PID");
        setDeadband(ControllerConst.DRIVE_THORTTLE_TRIGGER_VALUE);
    }

    public void setLeftSpeed(double speed) {
        leftGroup.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightGroup.set(speed);
    }

    public void setForwardSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public void setLeftVoltage(double volts) {
        volts = MathUtil.clamp(volts, -DriveConst.DRIVE_MAX_VOLTAGE, DriveConst.DRIVE_MAX_VOLTAGE);
        leftGroup.setVoltage(volts);
    }

    public void setRightVoltage(double volts) {
        volts = MathUtil.clamp(volts, -DriveConst.DRIVE_MAX_VOLTAGE, DriveConst.DRIVE_MAX_VOLTAGE);
        rightGroup.setVoltage(volts);
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        setLeftVoltage(leftVolts);
        setRightVoltage(rightVolts);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        driveTrain.arcadeDrive(xSpeed, zRotation, DriveConst.DRIVE_SQUARE_ARCADE);
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        driveTrain.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        driveTrain.tankDrive(leftSpeed, rightSpeed, DriveConst.DRIVE_SQUARE_TANK);
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        driveTrain.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }

    /**
     * Sets the deadband in the DifferentialDrive class. This will be used for the
     * arcadeDrive and tankDrive methods, but not for any of the setSpeed or
     * setVoltage methods.
     * 
     * @param deadband The deadband, between 0 and 1
     **/
    public void setDeadband(double deadband) {
        driveTrain.setDeadband(deadband);
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
     * This is the old PID for rotating to a gyro angle. It's recommended to use the
     * new {@link #gyroAngleToDriveDistances(double, double)} or
     * {@link #rotateGyroAngle(double currentGyroAngle, double goal)} instead.
     * 
     * @return The gyroPID instance
     */
    public GyroPID getGyroPID() {
        return gyroPID;
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

    @Override
    public void periodic() {
        super.periodic();
        m_odometry.update(Rotation2d.fromDegrees(getPigeonHeading()), Units.inchesToMeters(getDistanceRightEncoder()),
                Units.inchesToMeters(getDistanceLeftEncoder()));
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getPigeonHeading()));
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * 
     * @param currentGyroAngle The current gyro angle.
     * @param goal             The gyro angle you want to rotate to.
     * @return Array of drive distance: [left, right]
     */
    public double[] gyroAngleToDriveDistances(double currentGyroAngle, double goal) {
        return gyroToRotate.calculate(currentGyroAngle, goal);
    }

    /**
     * Automatically rotates to the intended gyro angle using left and right
     * encoders and PIDs. Needs to be called every cycle in the main control loop.
     * 
     * @param currentGyroAngle The current gyro angle
     * @param goal             The gyro angle you want to rotate to.
     */
    public void rotateGyroAngle(double currentGyroAngle, double goal) {
        // TODO: Cleanup and make a better way of doing this, going straight from PID to
        // speed controller groups
        double[] driveDistances = gyroAngleToDriveDistances(currentGyroAngle, goal);

        double leftCurrentDistance = getDistanceLeftEncoder();
        double rightCurrentDistance = getDistanceRightEncoder();

        double leftVoltage = leftPid.calculate(leftCurrentDistance, leftCurrentDistance + driveDistances[0]);
        double rightVoltage = rightPid.calculate(rightCurrentDistance, rightCurrentDistance + driveDistances[1]);

        setLeftVoltage(leftVoltage);
        setRightVoltage(rightVoltage);
    }

    /**
     * Overload for {@link #rotateGyroAngle(double currentGyroAngle, double goal)}
     * that automatically gets the current gyro angle
     * 
     * @param goal
     */
    public void rotateGyroAngle(double goal) {
        rotateGyroAngle(getPigeonHeading(), goal);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.setSmartDashboardType("DriveSubsystem");
    }


}