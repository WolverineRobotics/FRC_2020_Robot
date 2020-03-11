package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotMap;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax climb;
    private DutyCycleEncoder encoder;
    private TalonSRX climb_level;
    private DoubleSolenoid piston;

    private boolean climbLockFlag = false;

    private final double CLIMB_ENCODER_ZERO_ADD = -0.12;

    private double CLIMB_ENCODER_MAX = 4.3;
    private double CLIMB_UPPER_SOFT_LIMIT = 3.6;
    private double CLIMB_LOCK_ENCODER_COUNT = 1;
    private double CLIMB_ENCODER_SOFT_MIN = 1;
    private double CLIMB_ENCODER_MIN = 0.43;

    private double CLIMB_LOCK_ENCODER_FLAG = 1.5;

    private double SPEED_SOFT_REDUCTION = 0.5;

    private double encoderZero = 0;

    public ClimbSubsystem() {
        climb = new CANSparkMax(RobotMap.SpeedController.CLIMB, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(RobotMap.CLIMB_ENCODER);
        climb_level = new TalonSRX(RobotMap.SpeedController.CLIMB_LEVEL);
        piston = new DoubleSolenoid(RobotMap.Pneumatic.PCM, RobotMap.Pneumatic.CLIMB_LOCK_FORWARD, RobotMap.Pneumatic.CLIMB_LOCK_REVERSE);

        setLock(false);
    }

    public void setClimbSpeed(double speed) {
        double encoderPos = getClimbEncoderPosition();
        // if(encoderPos >= CLIMB_ENCODER_MAX && speed < 0) { // if climb go up and encoder is over soft limit
        //     speed = 0;
        // } else if(encoderPos <= CLIMB_ENCODER_MIN && speed >=0){ // if climb go down and encoder is below soft limit
        //     speed = 0;
        // } else if(encoderPos >= CLIMB_UPPER_SOFT_LIMIT && speed < 0){ // if climb encoder above soft limit and speed is down
        //     speed *= SPEED_SOFT_REDUCTION;
        // } else if(encoderPos <= CLIMB_ENCODER_SOFT_MIN && speed >=0){ // if climb encoder below soft limit and speed is up
        //     speed *= SPEED_SOFT_REDUCTION;
        // }

        if(speed > 0 && encoderPos <= CLIMB_LOCK_ENCODER_COUNT && climbLockFlag) { //if speed go down and encoder at lock 
            setLock(true);
        }
        
        if(!isClimbLockEngaged()) {
            climb.set(MathUtil.clamp(speed, -1, 1));
        } else {
            climb.set(0);
        }
    }

    public double getClimbSpeed() {
        return climb.get();
    }

    public void setClimbLevelSpeed(double speed) {
        climb_level.set(ControlMode.PercentOutput, MathUtil.clamp(speed, -1, 1));
    }

    public double getClimbLevelSpeed() {
        return climb_level.getSelectedSensorVelocity();
    }

    public double getClimbCurrent(){
        return climb.getOutputCurrent();
    }

    public boolean isClimbLockEngaged() {
        DoubleSolenoid.Value val = piston.get();
        if(val == Value.kForward) {
            return true;
        }
        return false;
    }

    public void toggleLock() {
        if(piston.get().equals(DoubleSolenoid.Value.kForward)) {
            piston.set(DoubleSolenoid.Value.kReverse);
            climbLockFlag = false;
        } else {
            piston.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void setLock(boolean toEngage) {
        if(toEngage) {
            piston.set(Value.kForward);
        } else {
            piston.set(Value.kReverse);
            climbLockFlag = false;
        }
    }

    public double getClimbEncoderPosition() {
        return encoder.get() + CLIMB_ENCODER_ZERO_ADD;
    }

    public double getClimbRawEncoderPosition(){
        return encoder.get();
    }

    public double getClimbEncoderDistance() {
        return encoder.getDistance();
    }

    @Override
    public void periodic() {

        if(getClimbEncoderPosition()> CLIMB_LOCK_ENCODER_FLAG ){
            this.climbLockFlag = true;
        }

        updateSDashboard();

    }

    private void updateSDashboard(){
        SmartDashboard.putNumber("Climb LEVEL Speed", getClimbLevelSpeed());
        SmartDashboard.putNumber("Climb Speed", getClimbSpeed());
        SmartDashboard.putNumber("[Climb] Climb Current", getClimbCurrent());
        SmartDashboard.putNumber("[Climb] Climb Encoder Position", getClimbEncoderPosition());
        SmartDashboard.putNumber("[Climb] Climb Encoder Distance", getClimbEncoderDistance());
    }

}