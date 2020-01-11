package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class NeoTestSubsystem extends SubsystemBase {

    private static final int deviceID = 1;
    private static final double MAX_NEO_VOLTAGE = 12;

    private CANSparkMax testMotor;

    public NeoTestSubsystem() {
        testMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    }

    public void setCurrentLimit(int currentLimit) {
        testMotor.setSmartCurrentLimit(currentLimit);
        testMotor.setSecondaryCurrentLimit(currentLimit);
    }

    public void setPower(double power) {
        testMotor.set(power);
    }

    public double getCurrent() {
        return testMotor.getOutputCurrent();
    }

    public void setVoltage(double voltage) {
        MathUtil.clamp(voltage, -MAX_NEO_VOLTAGE, MAX_NEO_VOLTAGE);
        testMotor.setVoltage(voltage);
    }

    public double getOutputCurrent() {
        return testMotor.getOutputCurrent();
    }

}