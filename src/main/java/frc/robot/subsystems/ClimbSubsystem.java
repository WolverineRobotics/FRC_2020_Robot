package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotMap;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax climb;
    private TalonSRX climb_level;

    public ClimbSubsystem() {
        climb = new CANSparkMax(RobotMap.SpeedController.CLIMB, MotorType.kBrushless);
        climb_level = new TalonSRX(RobotMap.SpeedController.CLIMB_LEVEL);
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

    @Override
    public void periodic() {
        updateSDashboard();
        SmartDashboard.putNumber("Climb LEVEL Speed", getClimbLevelSpeed());
        SmartDashboard.putNumber("Climb Speed", getClimbSpeed());
    }

    private void updateSDashboard(){
        SmartDashboard.putNumber("[Climb] Climb Current", getClimbCurrent());
    }

}