package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem c_drive;
    private SlewRateLimiter slew;

    private final boolean USE_SLEW_LIMITER = true;
    private double slew_rate = 1.2;

    public DefaultDriveCommand(DriveSubsystem drive) {
        c_drive = drive;
        addRequirements(drive);
        slew = new SlewRateLimiter(slew_rate);
        SmartDashboard.putNumber("[Drive - Default] Slew Rate ", slew_rate);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        c_drive.setForwardSpeed(0);
    }

    public double getSlewRate(){
        return this.slew_rate;
    }

    public void setSlewRate(double slewRate){
        if(this.slew_rate != slewRate){
            this.slew_rate = slewRate;
            slew = new SlewRateLimiter(slew_rate, slew.calculate(0));
        }
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        
        // Driver Left Stick Y and Driver Right Stick X
        double throttle = RobotContainer.getDriverController().getThrottle();
        double turn = RobotContainer.getDriverController().getTurn();

        double speedReduction = DriveConst.DRIVE_SPEED_REDUCTION_RATIO;


        if (RobotContainer.getDriverController().getFineControl()) {
            // If fine control is active.
            speedReduction = DriveConst.DRIVE_SPEED_REDUCTION_RATIO_FINE;
        } 

        arcadeDrive(throttle, turn, speedReduction, true);

    }

    private double calculateSlew(double throttle){
        return slew.calculate(throttle);
    }

    private void arcadeDrive(double throttle, double turn, double speedReduction) {
        arcadeDrive(throttle, turn, speedReduction, true);
        // c_drive.tankDrive(throttle, turn);
    }

    // private void tankDrive(double left, double right, double speedReduction){
    //     c_drive.tankDrive(throttle, turn);

    // }


    private void arcadeDrive(double throttle, double turn, double speedReduction, boolean invertTurn) {
        

        if (invertTurn) {
            turn = -turn;
        }
        throttle *= speedReduction;
        turn *= speedReduction;

        
        if(USE_SLEW_LIMITER){
            throttle = calculateSlew(throttle);
        }

        c_drive.arcadeDrive(throttle, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}