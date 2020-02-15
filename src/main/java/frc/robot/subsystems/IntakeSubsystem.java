package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    private DoubleSolenoid piston;

    public IntakeSubsystem() {
        sensor1 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_1);
        sensor2 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_2);
        sensor3 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_3);
        sensor4 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_4);
        sensor5 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_5);

        entry = new CANSparkMax(RobotMap.SpeedController.ENTRY, MotorType.kBrushless);
        curve = new CANSparkMax(RobotMap.SpeedController.CURVE, MotorType.kBrushless);
        vertical = new CANSparkMax(RobotMap.SpeedController.VERTICAL, MotorType.kBrushless);

        piston = new DoubleSolenoid(RobotMap.Pneumatic.INTAKE_FORWARD, RobotMap.Pneumatic.INTAKE_BACKWARD);

        piston.set(Value.kOff);
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

    public boolean isSensorOneActivated() {
        return sensor1.get();
    }

    public boolean isSensorTwoActivated() {
        return sensor2.get();
    }

    public boolean isSensorThreeActivated() {
        return sensor3.get();
    }

    public boolean isSensorFourActivated() {
        return sensor4.get();
    }

    public boolean isSensorFiveActivated() {
        return sensor5.get();
    }

    /**
     * Returns the amount of balls in the magazine, evaluated by the sensors.
     */
    public int getAmountOfBalls() {
        boolean[] sensors = {isSensorOneActivated(), isSensorTwoActivated(), isSensorThreeActivated(), isSensorFiveActivated(), isSensorFourActivated()};
        int count = 0;
        for(boolean b : sensors) {
            if(b) count++;
        }
        return count;
    }

    public boolean[] getSensors() {
        boolean[] sensors = {isSensorOneActivated(), isSensorTwoActivated(), isSensorThreeActivated(), isSensorFourActivated(), isSensorFiveActivated()};
        return sensors;
    }

    public boolean isIntakeOpen() {
        return piston.get() == Value.kForward;
    }

    public void setIntakePiston(boolean toOpen) {
        piston.set(toOpen ? Value.kForward : Value.kReverse);
    }

}