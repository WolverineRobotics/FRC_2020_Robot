package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.exceptions.NtEntryNullException;

public class CameraSubsystem extends SubsystemBase {

    /**
     * Default value when getting NetworkTableEntry of type double. Should be a
     * value that will never appear normally, to allow for checking of this value
     * and throwing an error if nessesary.
     */
    private final double ERR_DOUBLE = -99999;

    private NetworkTableInstance nt;
    private NetworkTable limelight;
    private NetworkTableEntry validTargets, xDegOff, yDegOff, targetArea, pipelineLatency;

    private NetworkTableEntry ledMode, camMode, snapshotMode;

    public CameraSubsystem() {
        nt = NetworkTableInstance.getDefault();
        limelight = nt.getTable("limelight");

        validTargets = limelight.getEntry("tv");
        xDegOff = limelight.getEntry("tx");
        yDegOff = limelight.getEntry("ty");
        targetArea = limelight.getEntry("ta");
        pipelineLatency = limelight.getEntry("tl");

        ledMode = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");
        snapshotMode = limelight.getEntry("snapshot");
    }

    public boolean hasValidTargets() {
        return validTargets.getBoolean(false);
    }

    public double getXDegOff() throws NtEntryNullException {
        double deg = xDegOff.getDouble(ERR_DOUBLE);
        if (deg == ERR_DOUBLE) {
            throw new NtEntryNullException(
                    "NetworkTable: Limelight: Horizontal Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    public double getYDegOff() throws NtEntryNullException {
        double deg = yDegOff.getDouble(ERR_DOUBLE);
        if (deg == ERR_DOUBLE) {
            throw new NtEntryNullException(
                    "NetworkTable: Limelight: Vertical Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    public double getTargetArea() throws NtEntryNullException {
        double area = targetArea.getDouble(ERR_DOUBLE);
        if (area == ERR_DOUBLE) {
            throw new NtEntryNullException("NetworkTable: Limelight: Area of target returned null");
        }
        return area;
    }

    public LimelightLEDMode getLedMode() throws NtEntryNullException {
        double modeNum = ledMode.getDouble(ERR_DOUBLE);
        if (modeNum == ERR_DOUBLE) {
            throw new NtEntryNullException("NetworkTable: Limelight: LED Mode returned null");
        }
        int mode = (int) Math.round(modeNum);

        return LimelightLEDMode.reverseLookup(mode);
    }

    public void setLedMode(LimelightLEDMode mode) {
        double modeNum = mode.getModeNum();
        ledMode.setValue(modeNum);
    }

    public LimelightVisionMode getCamMode() throws NtEntryNullException {
        double mode = camMode.getDouble(ERR_DOUBLE);
        if (mode == ERR_DOUBLE) {
            throw new NtEntryNullException(
                    "NetworkTable: Limelight: Camera mode (vision processing / drive) returned null");
        }
        int modeInt = (int) Math.round(mode);
        return LimelightVisionMode.reverseLookup(modeInt);
    }

    public void setCamMode(LimelightVisionMode mode) {
        camMode.setValue((double) mode.getModeNum());
    }

    public double getLatency() throws NtEntryNullException {
        double latency = pipelineLatency.getDouble(ERR_DOUBLE);
        if (latency == ERR_DOUBLE) {
            throw new NtEntryNullException("NetworkTable: Limelight: Pipeline latency returned null");
        }
        return latency;
    }

    public boolean isSnapshotMode() throws NtEntryNullException {
        double modeDouble = snapshotMode.getDouble(ERR_DOUBLE);
        if (modeDouble == ERR_DOUBLE) {
            throw new NtEntryNullException("NetworkTable: Limelight: Snapshot mode returned null");
        }
        if (Math.round(modeDouble) == 1) {
            return true;
        }
        return false;
    }

    public void setSnapshotMode(boolean enabled) {
        if (enabled) {
            snapshotMode.setDouble(1);
        } else {
            snapshotMode.setDouble(0);
        }
    }

    public enum LimelightVisionMode {
        VISION(0), DRIVER(1),;

        private static HashMap<Integer, LimelightVisionMode> map = new HashMap<>();
        private final int modeNum;

        private LimelightVisionMode(int modeNum) {
            this.modeNum = modeNum;
        }

        static {
            for (LimelightVisionMode ledMode : LimelightVisionMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        public int getModeNum() {
            return modeNum;
        }

        public static LimelightVisionMode reverseLookup(int value) {
            return map.get(value);
        }

    }

    /**
     * Enum for the mode of the LEDs of the limelight.
     */
    public enum LimelightLEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3),;

        private static HashMap<Integer, LimelightLEDMode> map = new HashMap<>();
        private final int modeNum;

        private LimelightLEDMode(int modeNum) {
            this.modeNum = modeNum;
        }

        static {
            for (LimelightLEDMode ledMode : LimelightLEDMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        public int getModeNum() {
            return modeNum;
        }

        public static LimelightLEDMode reverseLookup(int value) {
            return map.get(value);
        }
    }

}