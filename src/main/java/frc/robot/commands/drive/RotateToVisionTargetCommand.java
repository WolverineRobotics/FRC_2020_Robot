package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.exceptions.NTNullEntryException;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraMode;
import frc.robot.subsystems.CameraSubsystem.LEDMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MedianPercentileFilter;
import frc.robot.util.Util;

public class RotateToVisionTargetCommand extends CommandBase {
    private final CameraSubsystem s_camera;
    private final DriveSubsystem s_drive;
    private final MedianPercentileFilter xMedianFilter;
    private final MedianPercentileFilter yMedianFilter;

    private final int MEDIAN_FILTER_ENTRIES = 30;
    private final int CYCLES_UNTIL_CHECK_FINISHED = 10;
    private final double MIN_PERCENT_TARGETS_FOUND = 0.5;
    private final double MAX_IQR_X = 20;

    /**
     * The max acceptable error, in degree. If the error is below this, it will be
     * concidered on target (although it may still try to correct that error).
     */
    private final double TARGET_ERROR = 0.8;

    // Any targets with y-values below this value will be treated as false
    // positives.
    private final double Y_VALUE_CUTOFF = -10;

    private int totalCycles = 0;
    private int numFound = 0;

    private boolean finished = false;

    public RotateToVisionTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem) {
        s_camera = cameraSubsystem;
        s_drive = driveSubsystem;
        addRequirements(cameraSubsystem);
        addRequirements(driveSubsystem);
        xMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
        yMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
    }

    @Override
    public void initialize() {
        s_camera.setLEDMode(LEDMode.ON);
        s_camera.setCameraMode(CameraMode.VISION);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        try {
            double xDegOff = s_camera.getXDegOff();
            double yDegOff = s_camera.getYDegOff();

            // Test if target is within acceptable y values. If not, mark and return.
            if (yDegOff < Y_VALUE_CUTOFF) {
                noValidTargetFound(true);
                return;
            }

            double xGyroHeading = Util.addGyroValues(xDegOff, s_drive.getPigeonHeading());

            double xMedian, yMedian;

            xMedian = xMedianFilter.calculate(xGyroHeading);
            yMedian = yMedianFilter.calculate(yDegOff);

            // Pass xMedian to gyro PID

            s_drive.rotateGyroAngle(xMedian);

        } catch (NTNullEntryException exception) {
            System.out.println(exception.getMessage());
            noValidTargetFound(false);
        }

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
            if (xMedianFilter.getInterquartileRange() > MAX_IQR_X) {
                return true;
            }
        }
        return finished;
    }
}