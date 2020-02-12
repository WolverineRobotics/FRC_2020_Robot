package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Util;

public class SetFlyWheelSpeedTimedCommand extends CommandBase {

    private IntakeSubsystem s_intake;
    private double percentPower;
    private long timeMs;

    public SetFlyWheelSpeedTimedCommand(IntakeSubsystem s_intake, double percentPower, long timeMs) {
        this.s_intake = s_intake;
        this.percentPower = percentPower;
        this.timeMs = timeMs;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        s_intake.setShootSpeed(Util.getMotorLimits(percentPower));
    }

}