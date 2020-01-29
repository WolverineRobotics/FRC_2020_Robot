package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst;
import frc.robot.exceptions.NTNullEntryException;

public class CameraSubsystem extends SubsystemBase {

    private NetworkTable nt;
    private NetworkTableEntry validTargets, xDegOff, yDegOff, targetArea;
    private NetworkTableEntry targetSkew, targetShort, targetLong, targetHorizontal, targetVertical;
    private NetworkTableEntry ledMode, camMode, snapshotMode;
    private NetworkTableEntry pipelineLatency;

    public CameraSubsystem() {
        nt = NetworkTableInstance.getDefault().getTable("limelight");

        validTargets = nt.getEntry("tv");
        xDegOff = nt.getEntry("tx");
        yDegOff = nt.getEntry("ty");
        targetArea = nt.getEntry("ta");

        targetSkew = nt.getEntry("ts");
        targetShort = nt.getEntry("tshort");
        targetLong = nt.getEntry("tlong");
        targetHorizontal = nt.getEntry("thor");
        targetVertical = nt.getEntry("tvert");

        ledMode = nt.getEntry("ledMode");
        camMode = nt.getEntry("camMode");
        snapshotMode = nt.getEntry("snapshot");

        pipelineLatency = nt.getEntry("tl");
    }

    // ========================================================================
    // GETTERS
    // ========================================================================

    public boolean hasValidTargets() {
        return validTargets.getDouble(0) == 1;
    }

    public double getXDegOff() throws NTNullEntryException {
        final double deg = xDegOff.getDouble(RobotConst.VisionConst.ERROR);
        if (deg == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Horizontal Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    public double getYDegOff() throws NTNullEntryException {
        final double deg = yDegOff.getDouble(RobotConst.VisionConst.ERROR);
        if (deg == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Vertical Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    public double getTargetArea() throws NTNullEntryException {
        double area = targetArea.getDouble(RobotConst.VisionConst.ERROR);
        if (area == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Area of target returned null");
        }
        return area;
    }

    public LEDMode getLedMode() throws NTNullEntryException {
        final double modeNum = ledMode.getDouble(0);
        if (modeNum == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: LED Mode returned null");
        }
        final int mode = (int) Math.round(modeNum);

        return LEDMode.reverseLookup(mode);
    }

    public CameraMode getCamMode() throws NTNullEntryException {
        final double mode = camMode.getDouble(RobotConst.VisionConst.ERROR);
        if (mode == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Camera mode (vision processing / drive) returned null");
        }
        final int modeInt = (int) Math.round(mode);
        return CameraMode.reverseLookup(modeInt);
    }

    public double getLatency() throws NTNullEntryException {
        final double latency = pipelineLatency.getDouble(RobotConst.VisionConst.ERROR);
        if (latency == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Pipeline latency returned null");
        }
        return latency;
    }

    public boolean isSnapshotMode() throws NTNullEntryException {
        final double modeDouble = snapshotMode.getDouble(RobotConst.VisionConst.ERROR);
        if (modeDouble == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Snapshot mode returned null");
        }
        if (Math.round(modeDouble) == 1) {
            return true;
        }
        return false;
    }

    // ========================================================================
    // SETTERS
    // ========================================================================

    public void setSnapshotMode(final boolean enabled) {
        if (enabled) {
            snapshotMode.setDouble(1);
        } else {
            snapshotMode.setDouble(0);
        }
    }

    public void setLedMode(final LEDMode mode) {
        final double modeNum = mode.getModeNum();
        ledMode.setValue(modeNum);
    }

    public void setCamMode(final CameraMode mode) {
        camMode.setValue((double) mode.getModeNum());
    }

    /**
     * Enum for the different camera uses. VISION - used for vision
     * processing/calculating (programmer-side) DRIVER - disables processing and
     * opens stream (driver-side) See #setCamMode(CameraMode)
     */
    public enum CameraMode {
        VISION(0), DRIVER(1),;

        private static HashMap<Integer, CameraMode> map = new HashMap<>();
        private final int modeNum;

        private CameraMode(final int modeNum) {
            this.modeNum = modeNum;
        }

        static {
            for (final CameraMode ledMode : CameraMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        public int getModeNum() {
            return modeNum;
        }

        public static CameraMode reverseLookup(final int value) {
            return map.get(value);
        }

    }

    /**
     * Enum for the mode of the LEDs of the limelight. PIPELINE - uses the LED mode
     * currently in the pipeline OFF - turns LED off BLINK - blinks ON - turns LED
     * on see setLEDMode(LEDMode)
     */
    public enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3),;

        private static HashMap<Integer, LEDMode> map = new HashMap<>();
        private final int modeNum;

        static {
            for (final LEDMode ledMode : LEDMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        private LEDMode(final int modeNum) {
            this.modeNum = modeNum;
        }

        public int getModeNum() {
            return modeNum;
        }

        public static LEDMode reverseLookup(final int value) {
            return map.get(value);
        }
    }

}