package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.exceptions.NTNullEntryException;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraMode;
import frc.robot.subsystems.CameraSubsystem.LEDMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MedianPercentileFilter;

public class RotateToVisionTargetCommand extends CommandBase {
    private final CameraSubsystem m_camera;
    private final DriveSubsystem m_drive;
    private final MedianPercentileFilter xMedianFilter;
    private final MedianPercentileFilter yMedianFilter;

    private final int MEDIAN_FILTER_ENTRIES = 30;
    private final int CYCLES_UNTIL_CHECK_FINISHED = 10;
    private final double MIN_PERCENT_TARGETS_FOUND = 0.5;
    private final double MAX_IQR_X = 20;

    // Any targets with y-values below this value will be treated as false
    // positives.
    private final double Y_VALUE_CUTOFF = -10;

    private int totalCycles = 0;
    private int numFound = 0;

    private boolean finished = false;

    public RotateToVisionTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem) {
        m_camera = cameraSubsystem;
        m_drive = driveSubsystem;
        addRequirements(cameraSubsystem);
        addRequirements(driveSubsystem);
        xMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
        yMedianFilter = new MedianPercentileFilter(MEDIAN_FILTER_ENTRIES);
    }

    @Override
    public void initialize() {
        m_camera.setLEDMode(LEDMode.ON);
        m_camera.setCameraMode(CameraMode.VISION);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        try {
            double xDegOff = m_camera.getXDegOff();
            double yDegOff = m_camera.getYDegOff();

            // Test if target is within acceptable y values. If not, mark and return.
            if (yDegOff < Y_VALUE_CUTOFF) {
                noValidTargetFound(true);
                return;
            }

            double xGyroHeading = xDegOff + m_drive.getPigeonHeading();

            double xMedian, yMedian;

            xMedian = xMedianFilter.calculate(xDegOff);
            yMedian = yMedianFilter.calculate(yDegOff);

            // Pass xMedian to gyro PID

        } catch (NTNullEntryException exception) {
            System.out.println(exception.getMessage());
            noValidTargetFound(false);
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.setForwardSpeed(0);
        m_camera.setLEDMode(LEDMode.OFF);
        m_camera.setCameraMode(CameraMode.DRIVER);
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

        // Include code to end when at goal

        return finished;
    }
}