package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst.ShooterConst;
import frc.robot.constants.RobotMap;
import frc.robot.util.RevAbsoluteEncoder;
import frc.robot.util.UARTLidar;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax flywheel;
    private final CANEncoder flywheelEncoder;
    // private final CANPIDController flywheelPID;
    private final TalonSRX hood;
    private final RevAbsoluteEncoder hoodEncoder;
    private final UARTLidar lidar;

    // private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMaxRPM;

    public ShooterSubsystem() {
        flywheel = new CANSparkMax(RobotMap.SHOOTER_FLYWHEEL_MOTOR_ADDRESS, MotorType.kBrushless);
        flywheelEncoder = new CANEncoder(flywheel);
        // flywheelPID = new CANPIDController(flywheel);

        hood = new TalonSRX(RobotMap.SHOOTER_HOOD_MOTOR_ADDRESS);
        hoodEncoder = new RevAbsoluteEncoder(RobotMap.SHOOTER_HOOD_ENCODER_ADDRESS);

        lidar = new UARTLidar(RobotMap.LIDAR_PORT);
        SendableRegistry.add(this, "Shooter Subsystem");
    }

    @Override
    public void periodic() {
        lidar.recordDistance();
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

    /**
     * Gets the flywheel speed by getting the motor speed multiplied by the gear
     * ratio.
     * 
     * @return The flywheel speed, in RPM
     */
    public double getFlywheelSpeed() {
        return getFlywheelRawSpeed() * ShooterConst.SHOOTER_FLYWHEEL_GEAR_RATIO;
    }

    public int getLidarDistanceCM() {
        return lidar.getDistanceCM();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addDoubleProperty("[Shooter] Hood Position", this::getHoodEncoderPosition, null);
        builder.addDoubleProperty("[Shooter] Flywheel RPM", this::getFlywheelSpeed, null);
    }

}
