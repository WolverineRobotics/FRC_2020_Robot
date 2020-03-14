package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst.DriveConst;
import frc.robot.constants.RobotConst.PIDConst;
import frc.robot.exceptions.NTNullEntryException;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraMode;
import frc.robot.subsystems.CameraSubsystem.LEDMode;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToVisionTargetCommand extends RotateToHeadingProfiledCommand {
    private final CameraSubsystem s_camera;
    private final DriveSubsystem s_drive;
    // private final MedianPercentileFilter xMedianFilter;
    // private final MedianPercentileFilter yMedianFilter;

    private double kP = 0.016*1;
    private double kI = PIDConst.DRIVE_TURN_KI*6*10;
    private double kD = PIDConst.DRIVE_TURN_KD*3*4;

    // private final int MEDIAN_FILTER_ENTRIES = 10;
    protected final int CYCLES_UNTIL_CHECK_FINISHED = 10;
    private final double MIN_PERCENT_TARGETS_FOUND = 0.8;
    // private final double MAX_IQR_X = 20;

    /**
     * The max acceptable error, in degree. If the error is below this, it will be
     * concidered on target (although it may still try to correct that error).
     */
    private final double TARGET_ERROR = 0.9;

    // Any targets with y-values below this value will be treated as false
    // positives.
    // private final double Y_VALUE_CUTOFF = -20;

    protected int totalCycles = 0;
    protected int numFound = 0;

    private boolean finished = false;


    public RotateToVisionTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem) {
        // System.out.println("STARTING ROTATE TO VISION TARGET COMMAND =============================");
        super(driveSubsystem, 0);
        s_camera = cameraSubsystem;
        s_drive = driveSubsystem;
        addRequirements(cameraSubsystem);
        // addRequirements(driveSubsystem);
        
        // xMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
        // yMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
        s_drive.setDeadband(0);
    }

    @Override
    public void initialize() {
        super.initialize();
        s_camera.setLEDMode(LEDMode.ON);
        s_camera.setCameraMode(CameraMode.VISION);
        getController().setSetpoint(0);
        getController().setPID(kP, kI, kD);
        getController().setTolerance(0.6, 0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {
    //     // System.out.println("ROTATE TO VISION TARGET COMMAND");
    //     try {
    //         double xDegOff = s_camera.getXDegOff();
    //         // double yDegOff = s_camera.getYDegOff();

    //         // Test if target is within acceptable y values. If not, mark and return.
    //         // if (yDegOff < Y_VALUE_CUTOFF) {
    //         //     noValidTargetFound(true);
    //         //     return;
    //         // }

    //         // double xGyroHeading = Util.addGyroValues(xDegOff, s_drive.getPigeonHeading());

    //         // double xMedian, yMedian;

    //         // xMedian = xMedianFilter.calculate(xGyroHeading);
    //         // yMedian = yMedianFilter.calculate(yDegOff);

    //         // xMedian = xDegOff;
    //         // yMedian = yDegOff;

    //         // Pass xMedian to gyro PID

    //         // s_drive.rotateGyroAngle(xMedian);
            
    //         // double angleDifferance = Util.subtractGyroValues(xMedian, s_drive.getPigeonHeading());
    //         double angleDifferance = xDegOff;

    //         // double pGain = calculatePGain(angleDifferance);
    //         // pGain += 0.068 * Math.signum(pGain);
    //         // pGain = MathUtil.clamp(pGain, -.4, .4);

    //         // s_drive.arcadeDrive(0, - pGain, false);

    //     } catch (NTNullEntryException exception) {
    //         System.out.println(exception.getMessage());
    //         noValidTargetFound(false);
    //     }

    // }

    @Override
    public double getCurrentAngle() {
        try{
            return s_camera.getXDegOff();
        }catch(Exception e){
            System.err.println("No Vision Target Found");
            return 0;
        }
    }

    private double calculatePGain(double gyroError){
        return gyroError * kP;
    }

    public boolean isOnTarget() {
        // Could alternatively use the medianFilter for this
        try {
            double xError = Math.abs(s_camera.getXDegOff());
            return xError < TARGET_ERROR;
        } catch (NTNullEntryException e) {
            return false;
        }

    }

    private double updatePercentTargets(boolean found) {
        totalCycles++;
        if (found) {
            numFound++;
        }
        return numFound / totalCycles;
    }

    private double getPercentTargets() {
        return numFound / totalCycles;
    }

    /**
     * Called when no vision target was found.
     * 
     * @param invalidTarget True if a target was found but concidered invalid
     */
    private void noValidTargetFound(boolean invalidTarget) {
        updatePercentTargets(false);
    }

    /**
     * Call when you want this command to finish. If it can find a vision target, it
     * will continue rotating to it until this is called. If it can't find a vision
     * target, it will finish on its own
     */
    public void setFinished() {
        finished = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_drive.setForwardSpeed(0);
        s_drive.setDeadband(DriveConst.DRIVE_THORTTLE_TRIGGER_VALUE);
        s_camera.setLEDMode(LEDMode.OFF);
        s_camera.setCameraMode(CameraMode.DRIVER);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Code if no vision target
        if (totalCycles >= CYCLES_UNTIL_CHECK_FINISHED) {
            if (getPercentTargets() < MIN_PERCENT_TARGETS_FOUND) {
                return true;
            }
            // Ends if IQR is above a certain range
            // if (xMedianFilter.getInterquartileRange() > MAX_IQR_X) {
            //     return true;
            // }
        }
        return finished;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
