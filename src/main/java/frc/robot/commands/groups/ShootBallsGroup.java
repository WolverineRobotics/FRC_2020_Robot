package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.MoveBallsOneStageCommand;
import frc.robot.commands.intake.MoveBallsToShootCommand;
import frc.robot.commands.shootercommands.SetFlywheelShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBallsGroup extends ParallelRaceGroup {

    protected final MoveBallsToShootCommand c_moveBallsToShoot;
    protected final SetFlywheelShootCommand c_setFlywheelShoot;
    private boolean shootReady = true;
    protected double rpmSetpoint;
    private static final double DEFAULT_RPM = 4600;

    public ShootBallsGroup(IntakeSubsystem intake, ShooterSubsystem shooter, double rpm) {
        c_moveBallsToShoot = new MoveBallsToShootCommand(intake);
        c_setFlywheelShoot = new SetFlywheelShootCommand(shooter);
        addCommands(c_moveBallsToShoot);
        addCommands(c_setFlywheelShoot);    
        this.rpmSetpoint = rpm;
    }

    public ShootBallsGroup(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this(intake, shooter, DEFAULT_RPM);
    }

    public void setShootReady(boolean shoot) {
        // System.out.print("SHOOT BALLS - Set Shoot Ready: ");
        // if(shoot){
        //     System.out.println("True");
        // }else{
        //     System.out.println("False");
        // }
        this.shootReady = shoot;
    }

    @Override
    public void initialize() {
        super.initialize();
        c_setFlywheelShoot.setSetpointRPM(rpmSetpoint);
    }

    @Override
    public void execute() {
        // Passes if the flywheel is at speed from c_setFlywheelShoot to
        // c_moveBallsToShoot
        boolean flywheelAtSpeed = checkFlywheelAtSpeed();
        c_moveBallsToShoot.setFlywheelReady(flywheelAtSpeed && shootReady);

        // System.out.print("Command Group: Flywheel At Speed: ");
        // if(flywheelAtSpeed && shootReady){
        //     System.out.println("True");
        // }else{
        //     System.out.println("False");
        // }

        super.execute();
    }

    protected boolean checkFlywheelAtSpeed() {
        return c_setFlywheelShoot.isFlywheelAtSpeed();
    }

}