package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoTestSubsystem extends SubsystemBase{

    private static final int deviceID = 1;

    private CANSparkMax testMotor = new CANSparkMax(deviceID, MotorType.kBrushless);

    public void setCurrentLimit(int currentLimit){
        testMotor.setSmartCurrentLimit(currentLimit);
        testMotor.setSecondaryCurrentLimit(currentLimit);
    }

    public void setPower(double power){
        testMotor.set(power);
    }

    public double getCurrent(){
        return testMotor.getOutputCurrent();
        testMotor.setVoltage(outputVolts);
    }

}