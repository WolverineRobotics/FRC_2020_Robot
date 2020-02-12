package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap.ShooterMap;
import frc.robot.util.RevAbsoluteEncoder;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax flywheel;
    private final CANEncoder flywheelEncoder;
    private final TalonSRX hood;
    private final RevAbsoluteEncoder hoodEncoder;

    public ShooterSubsystem() {
        flywheel = new CANSparkMax(ShooterMap.SHOOTER_FLYWHEEL_MOTOR_ADDRESS, MotorType.kBrushless);
        flywheelEncoder = new CANEncoder(flywheel);
        hood = new TalonSRX(ShooterMap.SHOOTER_HOOD_MOTOR_ADDRESS);
        hoodEncoder = new RevAbsoluteEncoder(ShooterMap.SHOOTER_HOOD_ENCODER_ADDRESS);
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

    public void setHoodSpeed(double speed) {
        hood.set(TalonSRXControlMode.PercentOutput, speed);
    }

    /**
     * 
     * @return The flywheel speed, in RPM
     */
    public double getFlywheelSpeed() {
        return flywheelEncoder.getVelocity();
    }

}
