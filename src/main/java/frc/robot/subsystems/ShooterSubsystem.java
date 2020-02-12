package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap.ShooterMap;

public class ShooterSubsystem extends SubsystemBase{

    private final CANSparkMax flywheel;
    private final TalonSRX hood;

    public ShooterSubsystem(){
        flywheel = new CANSparkMax(ShooterMap.SHOOTER_FLYWHEEL_MOTOR_ADDRESS, MotorType.kBrushless);
        hood = new TalonSRX(ShooterMap.SHOOTER_HOOD_MOTOR_ADDRESS);
    }

}