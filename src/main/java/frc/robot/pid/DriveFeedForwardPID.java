package frc.robot.pid;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotConst.PidConst;
import frc.robot.util.Util;

/**
 * PID with feed forward for use with drivetrain. One instance for each set of
 * wheels, 2 total (left and right).
 */
public class DriveFeedForwardPID extends ProfiledPIDController {

    private static final double DEFAULT_KP = PidConst.DRIVE_FF_KP;
    private static final double DEFAULT_KI = PidConst.DRIVE_FF_KI;
    private static final double DEFAULT_KD = PidConst.DRIVE_FF_KD;

    private static final double kS = CharacterizationConst.KS_VOLTS;
    private static final double kV = CharacterizationConst.KS_VOLT_SECONDS_PER_METER;
    private static final double kA = CharacterizationConst.KS_VOLT_SECONDS_SQUARED_PER_METER;
    private static final Constraints constraints = new Constraints(CharacterizationConst.K_MAX_SPEED_METERS_PER_SECOND,
            CharacterizationConst.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    private static final double PID_ERROR_TOLERANCE = PidConst.DRIVE_PID_ERROR_TOLERANCE;

    private static final double MAX_VOLTAGE = DriveConst.DRIVE_MAX_VOLTAGE;

    private static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    /**
     * Uses the default PID gains in PidConst
     */
    public DriveFeedForwardPID() {
        this(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    public DriveFeedForwardPID(double kP, double kI, double kD) {
        super(kP, kI, kD, constraints);
        setIntegratorRange(-PidConst.MAX_INTEGRAL, PidConst.MAX_INTEGRAL);
        setTolerance(PID_ERROR_TOLERANCE);
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
        double pidVoltage = Util.toVoltage(pidPower, MAX_VOLTAGE);

        double feedForwardVoltage = feedForward.calculate(m_setpoint.velocity);

        return MathUtil.clamp(pidVoltage + feedForwardVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

}