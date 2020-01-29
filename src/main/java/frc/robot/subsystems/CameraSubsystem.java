package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst;
import frc.robot.exceptions.NTNullEntryException;

public class CameraSubsystem extends SubsystemBase {

    private final NetworkTable nt;
    private final NetworkTableEntry validTargets, xDegOff, yDegOff, targetArea, pipelineLatency;
    private final NetworkTableEntry ledMode, camMode, snapshotMode;

    public CameraSubsystem() {
        nt = NetworkTableInstance.getDefault().getTable("limelight");

        validTargets = nt.getEntry("tv");
        xDegOff = nt.getEntry("tx");
        yDegOff = nt.getEntry("ty");
        targetArea = nt.getEntry("ta");
        pipelineLatency = nt.getEntry("tl");

        ledMode = nt.getEntry("ledMode");
        camMode = nt.getEntry("camMode");
        snapshotMode = nt.getEntry("snapshot");
    }

    public boolean hasValidTargets() {
        return validTargets.getBoolean(false);
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

    public LimelightLEDMode getLedMode() throws NTNullEntryException {
        final double modeNum = ledMode.getDouble(0);
        if (modeNum == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: LED Mode returned null");
        }
        final int mode = (int) Math.round(modeNum);

        return LimelightLEDMode.reverseLookup(mode);
    }

    public void setLedMode(final LimelightLEDMode mode) {
        final double modeNum = mode.getModeNum();
        ledMode.setValue(modeNum);
    }

    public LimelightVisionMode getCamMode() throws NTNullEntryException {
        final double mode = camMode.getDouble(RobotConst.VisionConst.ERROR);
        if (mode == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Camera mode (vision processing / drive) returned null");
        }
        final int modeInt = (int) Math.round(mode);
        return LimelightVisionMode.reverseLookup(modeInt);
    }

    public void setCamMode(final LimelightVisionMode mode) {
        camMode.setValue((double) mode.getModeNum());
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

    public void setSnapshotMode(final boolean enabled) {
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

        private LimelightVisionMode(final int modeNum) {
            this.modeNum = modeNum;
        }

        static {
            for (final LimelightVisionMode ledMode : LimelightVisionMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        public int getModeNum() {
            return modeNum;
        }

        public static LimelightVisionMode reverseLookup(final int value) {
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

        static {
            for (final LimelightLEDMode ledMode : LimelightLEDMode.values()) {
                map.put(ledMode.modeNum, ledMode);
            }
        }

        private LimelightLEDMode(final int modeNum) {
            this.modeNum = modeNum;
        }

        public int getModeNum() {
            return modeNum;
        }

        public static LimelightLEDMode reverseLookup(final int value) {
            return map.get(value);
        }
    }

}