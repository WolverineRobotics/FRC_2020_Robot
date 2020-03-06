package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DrivePower;
import frc.robot.subsystems.DriveSubsystem.DriveVoltage;

/**
 * Given a heading and a distance, drives there using a profiled PID.
 */
public class DriveHeadingProfiledCommand extends CommandBase {


    private static final double TIME_PERIOD = 0.02;
    protected static final double DEFAULT_MAX_VOLTAGE = 10;

    private final DifferentialDriveKinematics m_kinematics;

    protected DriveSubsystem s_drive;

    private ProfiledPIDController pid_straight, pid_turn;
    private SimpleMotorFeedforward ff_drive;

    private double straightKp = PIDConst.DRIVE_STRAIGHT_KP;
    private double straightKi = PIDConst.DRIVE_STRAIGHT_KI;
    private double straightKd = PIDConst.DRIVE_STRAIGHT_KD;
    private double straightMaxVeloticy; // TODO
    private double straightMaxAcceleration; // TODO
    private Constraints straightConstraints = new Constraints(straightMaxVeloticy, straightMaxAcceleration);

    private double turnKp = PIDConst.DRIVE_TURN_KP;
    private double turnKi = PIDConst.DRIVE_TURN_KI;
    private double turnKd = PIDConst.DRIVE_TURN_KD;

    /**
     * The total constraints will be divided between turning and straight. The
     * constraints for the turning PID will be set to the total constraints and the
     * constraings for the straight PID will be calculated after getting the output
     * of the turning PID.
     */
    private Constraints totalConstrants = new Constraints();// TODO

    private double maxVoltage = 10;

    // Straight goal, in meters.
    private double straightGoal;
    // Turn goal, as a gyro heading.
    private double turnGoal;

    private double previousTurnLeftVelocity = 0;
    private DifferentialDriveWheelSpeeds previousWheelSpeeds = new DifferentialDriveWheelSpeeds();

    public DriveHeadingProfiledCommand(DriveSubsystem drive, double distance, double heading) {
        this(drive, distance, heading, DEFAULT_MAX_VOLTAGE);
    }

    public DriveHeadingProfiledCommand(DriveSubsystem drive, double distance, double heading, double maxVoltage) {
        super();
        this.s_drive = drive;
        this.maxVoltage = maxVoltage;

        m_kinematics = drive.getKinematics();

        ff_drive = new SimpleMotorFeedforward(CharacterizationConst.KS_VOLTS,
                CharacterizationConst.KV_VOLT_SECONDS_PER_METER,
                CharacterizationConst.KA_VOLT_SECONDS_SQUARED_PER_METER);

        pid_straight = new ProfiledPIDController(straightKp, straightKi, straightKd, straightConstraints);
        pid_turn = new ProfiledPIDController(turnKp, turnKi, turnKd, totalConstrants);

        pid_straight.setGoal(distance);
        pid_turn.setGoal(heading);

        pid_turn.enableContinuousInput(-180, 180);

        addRequirements(drive);
    }

    // Drive encoder distance
    protected double getStraightMeasurement() {
        return s_drive.getDistance();
    }

    // Pigeon heading
    protected double getTurnMeasurement() {
        return s_drive.getPigeonHeading();
    }

    protected double getStraightGoal() {
        return straightGoal;
    }

    protected double getTurnGoal() {
        return turnGoal;
    }

    /**
     * Used to calculate the turn acceleration in order to calculate the stragiht
     * constraints.
     * 
     * @param turnWheelSpeeds
     * @return
     */
    protected double calculateTurnAcceleration(DifferentialDriveWheelSpeeds turnWheelSpeeds) {
        double changeInTurnVelocity = turnWheelSpeeds.leftMetersPerSecond - previousTurnLeftVelocity;
        double acceleration_m_per_s_squared = changeInTurnVelocity * TIME_PERIOD;
        return acceleration_m_per_s_squared;
    }

    /**
     * Using the turn velocity, calculates the max acheivable straight velocity and
     * acceleration by subtracting from the total constraints
     * 
     * @param turnPower
     * @param turnSetpoint
     * @return Straight constraints, in m/s and m/(s^2)
     */
    private Constraints calculateStraightConstraints(double turnPower, State turnSetpoint) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics
                .toWheelSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(turnSetpoint.velocity)));
        double straightMaxVelocity = totalConstrants.maxVelocity - Math.abs(wheelSpeeds.leftMetersPerSecond);
        double straightMaxAcceleration = totalConstrants.maxAcceleration
                - Math.abs(calculateTurnAcceleration(wheelSpeeds));
        return new Constraints(straightMaxVelocity, straightMaxAcceleration);
    }

    private DrivePower arcadeToPower(double xSpeed, double zRotation) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        double leftPower = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
        double rightPower = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);
        return (new DrivePower(leftPower, rightPower));
    }

    private DriveVoltage powerToVoltage(DrivePower power) {
        double leftVoltage = power.leftPower * maxVoltage;
        double rightVoltage = power.rightPower * maxVoltage;
        return (new DriveVoltage(leftVoltage, rightVoltage));
    }

    private DriveVoltage calculateFeedForwardVoltage(DifferentialDriveWheelSpeeds wheelSpeeds) {

        // double leftAcceleration = (wheelSpeeds.leftMetersPerSecond - previousWheelSpeeds.leftMetersPerSecond)
        //         * TIME_PERIOD;
        // double rightAcceleration = (wheelSpeeds.rightMetersPerSecond - previousWheelSpeeds.rightMetersPerSecond)
        //         * TIME_PERIOD;

        double leftAcceleration = 0;
        double rightAcceleration = 0;

        double leftFFVoltage = ff_drive.calculate(wheelSpeeds.leftMetersPerSecond, leftAcceleration);
        double rightFFVoltage = ff_drive.calculate(wheelSpeeds.rightMetersPerSecond, rightAcceleration);

        return (new DriveVoltage(leftFFVoltage, rightFFVoltage));
    }

    @Override
    public void execute() {
        // Gets power and setpoint for turnPID
        double turnPower = pid_turn.calculate(getTurnMeasurement(), getTurnGoal());
        State turnSetpoint = pid_turn.getSetpoint();

        // Uses the turn power and constraints to calculate and set the
        // straightConstraints, based on the total constraints
        Constraints straighConstraints = calculateStraightConstraints(turnPower, turnSetpoint);
        pid_straight.setConstraints(straighConstraints);

        // Gets power and setpoint for straightPID
        double straightPower = pid_straight.calculate(getStraightMeasurement(), getStraightGoal());
        State straightSetpoint = pid_straight.getSetpoint();

        // Calculates the wheel speeds for the straight and turn setpoints
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(straightSetpoint.velocity, 0, Units.degreesToRadians(turnSetpoint.velocity)));

        // Changes the straight and turn power to left and right voltages
        DriveVoltage pidVoltage = powerToVoltage(arcadeToPower(straightPower, turnPower));

        // Calculates the required feed forward voltages for the left and right wheel
        // speed velocities
        DriveVoltage ffVoltage = calculateFeedForwardVoltage(wheelSpeeds);

        // Adds the pid and feed forward voltages together and clamps them
        DriveVoltage totalVoltage = DriveVoltage.addVoltage(pidVoltage, ffVoltage);
        totalVoltage = DriveVoltage.clampVoltage(totalVoltage, maxVoltage);

        // Sets the voltage
        s_drive.setVoltage(totalVoltage.leftVoltage, totalVoltage.rightVoltage);

        // Sets the previous velocities and wheel speeds.
        previousTurnLeftVelocity = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(0, 0, Units.degreesToRadians(turnSetpoint.velocity))).leftMetersPerSecond;
        previousWheelSpeeds = wheelSpeeds;

        System.out.println("Turn Power:"  +  turnPower);
        System.out.println("Straight Power:"  +  straightPower);
        System.out.println("Left PID Voltage" + pidVoltage.leftVoltage);
        System.out.println("Right PID Voltage" + pidVoltage.rightVoltage);
        System.out.println("Left FF Voltage" + ffVoltage.leftVoltage);
        System.out.println("Right FF Voltage" + ffVoltage.rightVoltage);



        updateSDashboard();
    }

    @Override
    public boolean isFinished() {
        return pid_straight.atGoal() && pid_turn.atGoal();
    }

    protected void updateSDashboard(){
        SmartDashboard.putData("[Drive Heading Profiled Command] Straight PID", pid_straight);
        SmartDashboard.putData("[Drive Heading Profiled Command] Turn PID", pid_turn);
        SmartDashboard.putNumber("[Drive Heading Profiled Command] Straight Goal", getStraightGoal());
        SmartDashboard.putNumber("[Drive Heading Profiled Command] Turn Goal", getTurnGoal());
    }


}
