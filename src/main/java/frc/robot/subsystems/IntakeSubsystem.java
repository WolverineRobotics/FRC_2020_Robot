package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private DigitalInput sensor1;
    private DigitalInput sensor2;
    private DigitalInput sensor3;
    private DigitalInput sensor4;
    private DigitalInput sensor5;

    private CANSparkMax entry; // main intake, entry point of ball
    private CANSparkMax curve; // curves ball into vertical conveyor
    private CANSparkMax vertical; // controls vertical conveyor
    private CANSparkMax shoot; // controls final stage of vertical conveyor (flywheel)
    private TalonSRX hood; // controls hood shooter

    public IntakeSubsystem() {
        sensor1 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_1);
        sensor2 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_2);
        sensor3 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_3);
        sensor4 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_4);
        sensor5 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_5);

        entry = new CANSparkMax(RobotMap.SpeedController.ENTRY, MotorType.kBrushless);
        curve = new CANSparkMax(RobotMap.SpeedController.CURVE, MotorType.kBrushless);
        vertical = new CANSparkMax(RobotMap.SpeedController.VERTICAL, MotorType.kBrushless);
        shoot = new CANSparkMax(RobotMap.SpeedController.SHOOT, MotorType.kBrushless);
        hood = new TalonSRX(RobotMap.SpeedController.HOOD);
    }

    public void setEntrySpeed(double speed) {
        entry.set(speed);
    }

    public void setCurveSpeed(double speed) {
        curve.set(speed);
    }

    public void setVerticalSpeed(double speed) {
        vertical.set(speed);
    }

    public void setShootSpeed(double speed) {
        shoot.set(speed);
    }

}