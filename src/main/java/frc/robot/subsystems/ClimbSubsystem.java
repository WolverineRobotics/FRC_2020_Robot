package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax climbMotor;
    private TalonSRX climbLevel;
    private DoubleSolenoid climbLock;
    private DigitalInput sensor;


    public ClimbSubsystem(){
        super();
		climbLock = new DoubleSolenoid(RobotMap.CLIMB_LOCK_FOWARD_ADDRESS, RobotMap.CLIMB_LOCK_RESERVE_ADDRESS);
        climbMotor = new CANSparkMax( RobotMap.CLIMB_MOTOR_ADDRESS, MotorType.kBrushless);
        climbLevel = new TalonSRX(RobotMap.CLIMB_LEVEL_ADDRESS);
    }

    public void setClimbSpeed(double speed){
        climbMotor.set(speed);
    }

    public void setClimbLevel(double speed){
        climbLevel.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setClimbLock(boolean forward){
        if (forward){
            climbLock.set(Value.kForward);
        }
        else{
            climbLock.set(Value.kReverse);
        }
    }

    public double getClimbSpeed(){
        return climbMotor.get();
    }

    public double getClimbLevel(){
        return climbLevel.getMotorOutputPercent();
    }

    public DoubleSolenoid.Value getClimbLock() {
        return climbLock.get();
    }

    public boolean getSensor(){
        return sensor.get();
    }



    @Override
    public void periodic(){

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.addDoubleProperty("[Climb] Motor Speed", this::getClimbSpeed, null);
        builder.addDoubleProperty("[Climb] Climb Level", this::getClimbLevel, null);
        builder.addStringProperty("[Climb]Climb Lock", () -> {return this.getClimbLock().toString();}, null);
    }
}