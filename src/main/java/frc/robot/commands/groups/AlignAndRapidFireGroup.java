package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.RotateToVisionTargetCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlignAndRapidFireGroup extends ParallelRaceGroup {

    private final RotateToVisionTargetCommand c_rotate;
    private final ShootBallsGroup c_shoot;
    private final CameraSubsystem s_camera;

    public AlignAndRapidFireGroup(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter,
            CameraSubsystem camera) {

        c_rotate = new RotateToVisionTargetCommand(camera, drive) {
            @Override
            public boolean isFinished() {
                return false;
            }
        };

        c_shoot = new RapidFireBallsGroup(intake, shooter);
        s_camera = camera;

        addCommands(c_rotate);
        addCommands(c_shoot);
    }

    public AlignAndRapidFireGroup(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter,
    CameraSubsystem camera, double flywheelRPM) {
        this(drive, intake, shooter, camera);
        c_shoot.c_setFlywheelShoot.setSetpointRPM(flywheelRPM);
    }

    @Override
    public void initialize() {
        super.initialize();
        c_shoot.setShootReady(false);
    }

    @Override
    public void execute() {
        super.execute();
        c_shoot.setShootReady(c_rotate.isOnTarget() && s_camera.hasValidTargets());
    }
}