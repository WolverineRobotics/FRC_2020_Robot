package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.constants.RobotConst.DriveConst.CharacterizationConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToHeadingProfiledCommand extends ProfiledPIDCommand {

    private DriveSubsystem s_drive;

    public RotateToHeadingProfiledCommand(DriveSubsystem drive, double headingGoal) {
        super(new ProfiledPIDController(PIDConst.DRIVE_TURN_KP, PIDConst.DRIVE_TURN_KI, PIDConst.DRIVE_TURN_KD,
                new TrapezoidProfile.Constraints(CharacterizationConst.K_MAX_TURN_DEG_PER_SECOND,
                        CharacterizationConst.K_MAX_TURN_ACCEL_DEG_PER_SECOND_SQUARED)),
                drive::getPigeonHeading, headingGoal,
                (turn, state) -> drive.turnStateFeedForward(turn, state,
                        new TrapezoidProfile.Constraints(CharacterizationConst.K_MAX_TURN_DEG_PER_SECOND,
                                CharacterizationConst.K_MAX_TURN_ACCEL_DEG_PER_SECOND_SQUARED)),
                drive);

        this.s_drive = drive;
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(PIDConst.DRIVE_TURN_TOLERANCE_DEG, PIDConst.DRIVE_TURN_TOLERANCE_DEG_PER_SECOND);

        SendableRegistry.addLW(getController(), "Rotate to Heading Profiled PID");
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

    public void setGoal(double gyroAngle){
        getController().setGoal(gyroAngle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Rotate to Heading Profiled Command");

    }

}