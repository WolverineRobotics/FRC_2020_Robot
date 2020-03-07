package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drive.RotateToVisionTargetCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlignAndShootGroup extends ParallelRaceGroup {

    private final RotateToVisionTargetCommand c_rotate;
    private final ShootBallsCommand c_shoot;

    public AlignAndShootGroup(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter,
            CameraSubsystem camera) {

        c_rotate = new RotateToVisionTargetCommand(camera, drive){
            @Override
            public boolean isFinished() {
                return false;
            }
        };

        c_shoot = new ShootBallsCommand(intake, shooter);
        addCommands(c_rotate, c_shoot);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        c_shoot.setShootReady(false);
    }

    @Override
    public void execute() {
        super.execute();
        c_shoot.setShootReady(c_rotate.isOnTarget());
    }
}