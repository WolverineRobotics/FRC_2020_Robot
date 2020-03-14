package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst.ShooterConst;
import frc.robot.constants.RobotMap;
import frc.robot.util.RevAbsoluteEncoder;
import frc.robot.util.Util;

public class ShooterSubsystem extends SubsystemBase {

    // TODO: Change value
    private final double FLYWHEEL_kP = 0.01;

    private final CANSparkMax flywheel;
    private final CANEncoder flywheelEncoder;
    // private final CANPIDController flywheelPID;
    private final TalonSRX hood;
    private final RevAbsoluteEncoder hoodEncoder;

    // private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMaxRPM;

    public ShooterSubsystem() {
        super();

        flywheel = new CANSparkMax(RobotMap.SHOOTER_FLYWHEEL_MOTOR_ADDRESS, MotorType.kBrushless);
        flywheelEncoder = new CANEncoder(flywheel);
        // flywheelPID = new CANPIDController(flywheel);

        hood = new TalonSRX(RobotMap.SHOOTER_HOOD_MOTOR_ADDRESS);
        hoodEncoder = new RevAbsoluteEncoder(RobotMap.SHOOTER_HOOD_ENCODER_ADDRESS,
                ShooterConst.HOOD_ENCODER_ZERO_POSITION);
                
        flywheel.setIdleMode(IdleMode.kCoast);

    }

    public void setFlywheelSpeed(double speed) {
        flywheel.set(speed);
    }

    public void setFlywheelVoltage(double voltage) {
        flywheel.setVoltage(voltage);
    }

    public int getHoodEncoderPosition() {
        return hoodEncoder.getEncoderPosition();
    }

    public boolean isHoodEncoderConnected() {
        return hoodEncoder.isConnected();
    }

    public void setHoodSpeed(double speed) {
        hood.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public double getFlywheelRawSpeed() {
        return flywheelEncoder.getVelocity();
    }

    public double getHoodError(int setpoint) {
        double error = setpoint - getHoodEncoderPosition();
        error = Util.getContinuousError(error, hoodEncoder.COUNTS_PER_REVOLUTION);
        return error;
    }

    public double getFlywheelVoltage() {
        return flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    }

    public double getFlywheelCurrent() {
        return flywheel.getOutputCurrent();
    }

    /**
     * Gets the flywheel speed by getting the motor speed multiplied by the gear
     * ratio.
     * 
     * @return The flywheel speed, in RPM
     */
    public double getFlywheelSpeed() {
        return getFlywheelRawSpeed() * ShooterConst.SHOOTER_FLYWHEEL_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpmSetpoint) {
        setFlywheelRPM(rpmSetpoint, 0);
    }

    public void setFlywheelRPM(double rpmSetpoint, double basePower) {
        setFlywheelRPM(rpmSetpoint, basePower, FLYWHEEL_kP);
    }

    public void setFlywheelRPM(double rpmSetpoint, double basePower, double kP) {
        double error = rpmSetpoint - getFlywheelSpeed();
        double proportional = error * kP;
        setFlywheelSpeed(proportional + basePower);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addDoubleProperty("[Shooter] Hood Position", this::getHoodEncoderPosition, null);
        builder.addDoubleProperty("[Shooter] Flywheel RPM", this::getFlywheelSpeed, null);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putData(this);
        updateSDashboard();
    }

    private void updateSDashboard() {
        SmartDashboard.putNumber("[Shooter] Hood Position", this.getHoodEncoderPosition());
        SmartDashboard.putNumber("[Shooter] Flywheel RPM", this.getFlywheelSpeed());
        SmartDashboard.putNumber("[Shooter] Flywheel Voltage", this.getFlywheelVoltage());
        SmartDashboard.putNumber("[Shooter] Flywheel Current", this.getFlywheelCurrent());

    }

}
