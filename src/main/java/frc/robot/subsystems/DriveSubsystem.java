package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.ControllerConst;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.Drive;
import frc.robot.constants.RobotMap.Pneumatic;
import frc.robot.pid.DriveFeedForwardPID;
import frc.robot.pid.GyroPID;
import frc.robot.pid.GyroToRotate;
import frc.robot.util.Util;

public class DriveSubsystem extends SubsystemBase {


    public static class DrivePower {

        public final double leftPower;
        public final double rightPower;

        public DrivePower(double leftPower, double rightPower) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }
    }

    public static class DriveVoltage {

        public final double leftVoltage;
        public final double rightVoltage;

        public DriveVoltage(double leftVoltage, double rightVoltage) {
            this.leftVoltage = leftVoltage;
            this.rightVoltage = rightVoltage;
        }

        public static DriveVoltage addVoltage(DriveVoltage voltage1, DriveVoltage voltage2) {
            double leftVoltage = voltage1.leftVoltage + voltage2.leftVoltage;
            double rightVoltage = voltage1.rightVoltage + voltage2.rightVoltage;
            return (new DriveVoltage(leftVoltage, rightVoltage));
        }

        public static DriveVoltage clampVoltage(DriveVoltage inputVoltage, double maxVoltage) {
            double leftVoltage = MathUtil.clamp(inputVoltage.leftVoltage, -maxVoltage, maxVoltage);
            double rightVoltage = MathUtil.clamp(inputVoltage.rightVoltage, -maxVoltage, maxVoltage);
            return (new DriveVoltage(leftVoltage, rightVoltage));
        }

    }

    private CANSparkMax leftMaster, leftSlave1, leftSlave2, rightMaster, rightSlave1, rightSlave2;
    private Encoder leftEncoder, rightEncoder;

    private SpeedControllerGroup leftGroup, rightGroup;
    private DifferentialDrive driveTrain;

    /**
     * High gear is forward, low gear is reverse
     */
    // private DoubleSolenoid gearShifter;

    private DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveOdometry m_odometry;

    private SimpleMotorFeedforward feedForward;

    private AHRS navX;
    private PigeonIMU pigeon;

    public DriveFeedForwardPID leftPid;
    public DriveFeedForwardPID rightPid;

    private GyroToRotate gyroToRotate;

    private GyroPID gyroPID;

    private PowerDistributionPanel pdp;

    public DriveSubsystem() {
        super();

        leftMaster = new CANSparkMax(RobotMap.DRIVE_LEFT_MOTOR_MASTER_ADDRESS, MotorType.kBrushless);
        leftSlave1 = new CANSparkMax(RobotMap.DRIVE_LEFT_MOTOR_SLAVE_ADDRESS_1, MotorType.kBrushless);
        leftSlave2 = new CANSparkMax(RobotMap.DRIVE_LEFT_MOTOR_SLAVE_ADDRESS_2, MotorType.kBrushless);

        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);

        rightMaster = new CANSparkMax(RobotMap.DRIVE_RIGHT_MOTOR_MASTER_ADDRESS, MotorType.kBrushless);
        rightSlave1 = new CANSparkMax(RobotMap.DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS_1, MotorType.kBrushless);
        rightSlave2 = new CANSparkMax(RobotMap.DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS_2, MotorType.kBrushless);

        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);

        leftMaster.setIdleMode(IdleMode.kBrake);
        leftSlave1.setIdleMode(IdleMode.kBrake);
        leftSlave2.setIdleMode(IdleMode.kBrake);

        rightMaster.setIdleMode(IdleMode.kBrake);
        rightSlave1.setIdleMode(IdleMode.kBrake);
        rightSlave2.setIdleMode(IdleMode.kBrake);

        leftGroup = new SpeedControllerGroup(leftMaster);
        rightGroup = new SpeedControllerGroup(rightMaster);
        rightGroup.setInverted(true);
        leftGroup.setInverted(true);


        driveTrain = new DifferentialDrive(leftGroup, rightGroup);

        leftEncoder = new Encoder(Drive.DRIVE_LEFT_ENCODER_A, Drive.DRIVE_LEFT_ENCODER_B);
        rightEncoder = new Encoder(RobotMap.Drive.DRIVE_RIGHT_ENCODER_A, RobotMap.Drive.DRIVE_RIGHT_ENCODER_B);

        rightEncoder.setReverseDirection(true);

        leftEncoder.setDistancePerPulse(1/DriveConst.DRIVE_ENCODER_COUNTS_PER_METER);
        rightEncoder.setDistancePerPulse(1/DriveConst.DRIVE_ENCODER_COUNTS_PER_METER);

        // gearShifter = new DoubleSolenoid(Pneumatic.DRIVE_HIGH_GEAR_ADDRESS, Pneumatic.DRIVE_LOW_GEAR_ADDRESS);

        navX = new AHRS(Port.kMXP);
        pigeon = new PigeonIMU(RobotMap.DRIVE_PIGEON_IMU_ADDRESS);

        gyroPID = new GyroPID(PIDConst.GYRO_KP, PIDConst.GYRO_KI, PIDConst.GYRO_KD);

        m_kinematics = new DifferentialDriveKinematics(CharacterizationConst.K_TRACKWIDTH_METERS);
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getPigeonHeading()));

        feedForward = new SimpleMotorFeedforward(CharacterizationConst.KS_VOLTS,
                CharacterizationConst.KV_VOLT_SECONDS_PER_METER,
                CharacterizationConst.KA_VOLT_SECONDS_SQUARED_PER_METER);

        leftPid = new DriveFeedForwardPID();
        rightPid = new DriveFeedForwardPID();

        pigeon.setYaw(0);
        pigeon.setFusedHeading(0);

        gyroToRotate = new GyroToRotate(CharacterizationConst.K_TRACKWIDTH_METERS);

        pdp = new PowerDistributionPanel();

        setDeadband(DriveConst.DRIVE_THORTTLE_TRIGGER_VALUE);

        SendableRegistry.addLW(leftPid, "[Drive] Left PID");
        SendableRegistry.addLW(rightPid, "[Drive] Right PID");
    }

    // public void setHighGear() {
    //     gearShifter.set(Value.kForward);
    // }

    // public void setLowGear() {
    //     gearShifter.set(Value.kReverse);
    // }

    public void setLeftSpeed(double speed) {
        leftGroup.set(-speed);
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
        rightGroup.setVoltage(-volts);
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

    public double getLeftVoltage() {
        return (leftMaster.getAppliedOutput() * leftMaster.getBusVoltage()) +  (leftSlave1.getAppliedOutput()*leftSlave1.getBusVoltage()) + (leftSlave2.getAppliedOutput()*leftSlave2.getBusVoltage());
    }

    public double getRightVoltage() {
        return (rightMaster.getAppliedOutput() * rightMaster.getBusVoltage() + (rightSlave1.getAppliedOutput()*rightSlave1.getBusVoltage()) + (rightSlave2.getAppliedOutput()*rightSlave2.getBusVoltage()));
    }

    /**
     * 
     * @return the average current of the 3 motors on the left side
     */
    public double getLeftCurent() {
        return (leftMaster.getOutputCurrent() + leftSlave1.getOutputCurrent() + leftSlave2.getOutputCurrent());
    }

    /**
     * 
     * @return the average current of the 3 motors on the right side.
     */
    public double getRightCurent() {
        return (rightMaster.getOutputCurrent() + rightSlave1.getOutputCurrent() + rightSlave2.getOutputCurrent());
    }

    /**
     * 
     * @return left side velocity in meters per second.
     */
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    /**
     * 
     * @return right side velocity in meters per second.
     */
    public double getRightVelocity() {
        return rightEncoder.getRate();
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
        return (leftMaster.get());
    }

    public double getRightSpeed() {
        return (rightMaster.get());
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
        // double[] ypr = new double[3];
        // pigeon.getYawPitchRoll(ypr);
        // return ypr[0];
        double heading = pigeon.getFusedHeading();
        heading %= 360;
        if(heading < 0){
            heading += 360;
        }
        return heading;
    }

    /**
     * resets the gyro heading
     */
    public void resetNavxHeading() {
        navX.reset();
    }

    public DifferentialDriveKinematics getKinematics(){
        return m_kinematics;
    }

    @Override
    public void periodic() {
        super.periodic();
        m_odometry.update(Rotation2d.fromDegrees(getPigeonHeading()), getDistanceRightEncoder(),
                getDistanceLeftEncoder());
        updateSDashboard();
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
     * Takes the turn and state from a rotational profiled pid and applies a feed forward to it.
     * @param turn turn power from pid
     * @param state current state, in degrees and degrees per second
     * @param constraints in max degrees per second and max degrees per second squared
     */
    public void turnStateFeedForward(double turn, State state, Constraints constraints) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(0,0,Units.degreesToRadians(state.velocity)));
        
        //May need to invert
        double ffPowerVolts = feedForward.calculate(wheelSpeeds.rightMetersPerSecond);

        double ffSpeed =  Util.voltageToSpeed(ffPowerVolts, pdp.getVoltage());  

        turn += ffSpeed;
        arcadeDrive(0, turn, false);
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
    
    public void resetPigeonHeading(){
        pigeon.setFusedHeading(0);
        pigeon.setYaw(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("DriveSubsystem");

        builder.addDoubleProperty("[Drive] Left Speed", this::getLeftSpeed, null);
        builder.addDoubleProperty("[Drive] Right Speed", this::getRightSpeed, null);
        builder.addDoubleProperty("[Drive] Left Distance Encoder", this::getDistanceLeftEncoder, null);
        builder.addDoubleProperty("[Drive] Right Distance Encoder", this::getDistanceRightEncoder, null);
        builder.addDoubleProperty("[Drive] Left Voltage", this::getLeftVoltage, null);
        builder.addDoubleProperty("[Drive] Right Voltage", this::getRightVoltage, null);
        builder.addDoubleProperty("[Drive] Left Current", this::getLeftCurent, null);
        builder.addDoubleProperty("[Drive] Right Current", this::getRightCurent, null);
        builder.addDoubleProperty("[Drive] Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("[Drive] Right Velocity", this::getRightVelocity, null);

    }

    private void updateSDashboard(){
        SmartDashboard.putNumber("[Drive] Pigeon Heading", this.getPigeonHeading());
        SmartDashboard.putNumber("[Drive] Left Speed", this.getLeftSpeed());
        SmartDashboard.putNumber("[Drive] Right Speed", this.getRightSpeed());
        SmartDashboard.putNumber("[Drive] Left Encoder", this.getRawLeftEncoder());
        SmartDashboard.putNumber("[Drive] Right Encoder", this.getRawRightEncoder());

        SmartDashboard.putNumber("[Drive] Left Distance Encoder", this.getDistanceLeftEncoder());
        SmartDashboard.putNumber("[Drive] Right Distance Encoder", this.getDistanceRightEncoder());
        SmartDashboard.putNumber("[Drive] Left Voltage", this.getLeftVoltage());
        SmartDashboard.putNumber("[Drive] Right Voltage", this.getRightVoltage());
        SmartDashboard.putNumber("[Drive] Left Current", this.getLeftCurent());
        SmartDashboard.putNumber("[Drive] Right Current", this.getRightCurent());
        SmartDashboard.putNumber("[Drive] Left Velocity", this.getLeftVelocity());
        SmartDashboard.putNumber("[Drive] Right Velocity", this.getRightVelocity());

    }


}
