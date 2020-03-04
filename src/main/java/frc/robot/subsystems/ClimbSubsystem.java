package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotMap;
import frc.robot.util.RevAbsoluteEncoder;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax climb;
    private RevAbsoluteEncoder encoder;
    private TalonSRX climb_level;
    private DoubleSolenoid piston;

    public ClimbSubsystem() {
        climb = new CANSparkMax(RobotMap.SpeedController.CLIMB, MotorType.kBrushless);
        encoder = new RevAbsoluteEncoder(7);
        climb_level = new TalonSRX(RobotMap.SpeedController.CLIMB_LEVEL);
        piston = new DoubleSolenoid(3, RobotMap.Pneumatic.CLIMB_LOCK_FORWARD, RobotMap.Pneumatic.CLIMB_LOCK_REVERSE);
    }

    public void setClimbSpeed(double speed) {
        climb.set(MathUtil.clamp(speed, -1, 1));
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

    public void toggleLock() {
        if(piston.get().equals(DoubleSolenoid.Value.kForward)) {
            piston.set(DoubleSolenoid.Value.kReverse);
        } else {
            piston.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void setLock(boolean toEngage) {
        if(toEngage) {
            piston.set(Value.kForward);
        } else {
            piston.set(Value.kReverse);
        }
    }

    public int getClimbEncoderPosition() {
        return encoder.getEncoderPosition();
    }

    @Override
    public void periodic() {
        updateSDashboard();
    }

    private void updateSDashboard(){
        SmartDashboard.putNumber("Climb LEVEL Speed", getClimbLevelSpeed());
        SmartDashboard.putNumber("Climb Speed", getClimbSpeed());
        SmartDashboard.putNumber("[Climb] Climb Current", getClimbCurrent());
        SmartDashboard.putNumber("[Climb] Climb Encoder", getClimbEncoderPosition());
    }

}