package frc.robot.pid;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.util.Util;

/**
 * PID with feed forward for use with drivetrain. One instance for each set of
 * wheels, 2 total (left and right).
 */
public class DriveFeedForwardPID extends ProfiledPIDController {

    private static final double DEFAULT_KP = PIDConst.DRIVE_FF_KP;
    private static final double DEFAULT_KI = PIDConst.DRIVE_FF_KI;
    private static final double DEFAULT_KD = PIDConst.DRIVE_FF_KD;

    private static final double kS = CharacterizationConst.KS_VOLTS;
    private static final double kV = CharacterizationConst.KV_VOLT_SECONDS_PER_METER;
    private static final double kA = CharacterizationConst.KA_VOLT_SECONDS_SQUARED_PER_METER;

    private static final double DEFAULT_PID_ERROR_TOLERANCE = PIDConst.DRIVE_PID_ERROR_TOLERANCE;

    private static final double DEFAULT_MAX_VOLTAGE = DriveConst.DRIVE_MAX_VOLTAGE;

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    private double maxVoltage;
    private double PIDErrorTolerance;

    /**
     * Uses the default PID gains in PIDConst
     */
    public DriveFeedForwardPID() {
        this(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    public DriveFeedForwardPID(double kP, double kI, double kD) {
        this(kP, kI, kD, new Constraints(CharacterizationConst.K_MAX_SPEED_METERS_PER_SECOND,
                CharacterizationConst.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
    }

    public DriveFeedForwardPID(double kP, double kI, double kD, double maxVelocity) {
        this(kP, kI, kD,
                new Constraints(maxVelocity, CharacterizationConst.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
    }

    public DriveFeedForwardPID(double kP, double kI, double kD, Constraints constraints) {
        this(kP, kI, kD, constraints, DEFAULT_MAX_VOLTAGE);
    }

    public DriveFeedForwardPID(double kP, double kI, double kD, Constraints constraints, double maxVoltage) {
        this(kP, kI, kD, constraints, maxVoltage, DEFAULT_PID_ERROR_TOLERANCE);
    }

    public DriveFeedForwardPID(double kP, double kI, double kD, Constraints constraints, double maxVoltage,
            double PIDErrorTolerance) {
        super(kP, kI, kD, constraints);

        this.maxVoltage = maxVoltage;
        this.PIDErrorTolerance = PIDErrorTolerance;
        setIntegratorRange(-PIDConst.MAX_INTEGRAL, PIDConst.MAX_INTEGRAL);
        setTolerance(PIDErrorTolerance);
    }

    /**
     * Needs to be called every cycle of the main control loop. The
     * {@link #reset(double, double)} and {@link #setGoal(double)} methods should be
     * called to reset the PID and set the desired goal before calling this for the
     * first time.
     * 
     * @param measurement The current encoder distance measurement, in meters
     * 
     * @return The voltage to set the speed controllers to
     */
    @Override
    public double calculate(double measurement) {
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(getPeriod());

        double pidPower = m_controller.calculate(measurement, m_setpoint.position);
        double pidVoltage = Util.toVoltage(pidPower, maxVoltage);

        double feedForwardVoltage = feedForward.calculate(m_setpoint.velocity);

        return MathUtil.clamp(pidVoltage + feedForwardVoltage, -maxVoltage, maxVoltage);
    }

    public double getMaxVoltage() {
        return maxVoltage;
    }

    public void setMaxVoltage(double maxVoltage) {
        this.maxVoltage = maxVoltage;
    }

    public double getPIDErrorTolerance() {
        return PIDErrorTolerance;
    }

    public double getMaxSpeed() {
        return m_constraints.maxVelocity;
    }

    public void setMaxSpeed(double maxSpeedMetersPerSecond) {
        double oldAcceleration = m_constraints.maxAcceleration;
        Constraints newConstraints = new Constraints(maxSpeedMetersPerSecond, oldAcceleration);
        setConstraints(newConstraints);
    }

    public double getMaxAcceleration() {
        return m_constraints.maxAcceleration;
    }

    public void setMaxAcceleration(double maxAccelerationMetersPerSecondSquared) {
        double oldVelocity = m_constraints.maxVelocity;
        Constraints newConstraints = new Constraints(oldVelocity, maxAccelerationMetersPerSecondSquared);
        setConstraints(newConstraints);
    }

    public void setPIDErrorTolerance(double PIDErrorTolerance) {
        this.PIDErrorTolerance = PIDErrorTolerance;
        setTolerance(PIDErrorTolerance);
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("DriveFeedForwardPID");
        builder.addDoubleProperty("Max Voltage", this::getMaxVoltage, this::setMaxVoltage);
        builder.addDoubleProperty("Max Velocity", this::getMaxSpeed, this::setMaxSpeed);
        builder.addDoubleProperty("Max Acceleration", this::getMaxAcceleration, this::setMaxAcceleration);
    }

}