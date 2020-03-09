package frc.robot.commands.groups;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RapidFireBallsGroup extends ShootBallsGroup {

    private final double RPM_SETPOINT = 4600;
    private boolean hasRPMBeenReached = false;

    public RapidFireBallsGroup(IntakeSubsystem intake, ShooterSubsystem shooter) {
        super(intake, shooter);
        c_setFlywheelShoot.setSetpointRPM(RPM_SETPOINT);
    }

    @Override
    protected boolean checkFlywheelAtSpeed() {
        if (hasRPMBeenReached) {
            return true;
        }
        if (super.checkFlywheelAtSpeed()) {
            hasRPMBeenReached = true;
            return true;
        }
        return false;
    }
}