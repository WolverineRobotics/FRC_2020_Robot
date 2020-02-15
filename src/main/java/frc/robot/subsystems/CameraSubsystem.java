package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst;
import frc.robot.constants.RobotConst.VisionConst;
import frc.robot.exceptions.NTNullEntryException;

public class CameraSubsystem extends SubsystemBase {

    private NetworkTable nt;
    private NetworkTableEntry validTargets, xDegOff, yDegOff, targetArea;
    private NetworkTableEntry targetSkew, targetShort, targetLong, targetHorizontal, targetVertical;
    private NetworkTableEntry ledMode, camMode, snapshotMode;
    private NetworkTableEntry pipelineLatency;

    public CameraSubsystem() {
        super();

        nt = NetworkTableInstance.getDefault().getTable("limelight");

        // NetworkTableAPI docs:
        // http://docs.limelightvision.io/en/latest/networktables_api.html#advanced-usage-with-raw-contours
        validTargets = nt.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
        xDegOff = nt.getEntry("tx"); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees |
                                     // LL2: -29.8 to 29.8 degrees)
        yDegOff = nt.getEntry("ty"); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
                                     // | LL2: -24.85 to 24.85 degrees)
        targetArea = nt.getEntry("ta"); // Target Area (0% of image to 100% of image)

        targetSkew = nt.getEntry("ts"); // Skew or rotation (-90 degrees to 0 degrees)
        targetShort = nt.getEntry("tshort"); // Sidelength of shortest side of the fitted bounding box (pixels)
        targetLong = nt.getEntry("tlong"); // Sidelength of longest side of the fitted bounding box (pixels)
        targetHorizontal = nt.getEntry("thor"); // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
        targetVertical = nt.getEntry("tvert"); // Vertical sidelength of the rough bounding box (0 - 320 pixels)

        ledMode = nt.getEntry("ledMode"); // Sets limelight’s LED state See Enum "LEDMode"
        camMode = nt.getEntry("camMode"); // Sets limelight’s operation mode. See Enum "CameraMode"
        snapshotMode = nt.getEntry("snapshot"); // Allows users to take snapshots during a match

        pipelineLatency = nt.getEntry("tl"); // The pipeline’s latency contribution (ms) Add at least 11ms for image
                                             // capture latency.
    }

    // ========================================================================
    // GETTERS
    // ========================================================================

    /**
     * Returns true if there are any valid targets. CameraMode must be set to
     * LEDMode.VISION to vision process.
     */
    public boolean hasValidTargets() {
        return validTargets.getDouble(0) == 1;
    }

    /**
     * Returns the amount of degrees off horizontally the camera's center is to the
     * target crosshair.
     * 
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees |
     * LL2: -29.8 to 29.8 degrees)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table
     */
    public double getXDegOff() throws NTNullEntryException {
        final double deg = xDegOff.getDouble(RobotConst.VisionConst.ERROR);
        if (deg == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Horizontal Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    /**
     * Returns the amount of degrees off vertically the camera's center is to the
     * target crosshair.
     * 
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
     * | LL2: -24.85 to 24.85 degrees)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table
     * 
     */
    public double getYDegOff() throws NTNullEntryException {
        final double deg = yDegOff.getDouble(RobotConst.VisionConst.ERROR);
        if (deg == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Vertical Offset From Crosshair To Target returned null");
        }
        return deg;
    }

    /**
     * Returns the target's surface area in how much percent of the screen it is
     * taking up (0.0 - 1.0) i assume is the range ~ someone confirm uwu
     * 
     * Target Area (0% of image to 100% of image)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table
     */
    public double getTargetArea() throws NTNullEntryException {
        double area = targetArea.getDouble(RobotConst.VisionConst.ERROR);
        if (area == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Area of target returned null");
        }
        return area;
    }

    /**
     * Returns the LEDMode enum in CameraSubsytem#LEDMode
     * 
     * See enum "LEDMode" for more info
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table
     */
    public LEDMode getLEDMode() throws NTNullEntryException {
        final double modeNum = ledMode.getDouble(RobotConst.VisionConst.ERROR);
        if (modeNum == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: LED Mode returned null");
        }
        final int mode = (int) Math.round(modeNum);

        return LEDMode.reverseLookup(mode);
    }

    /**
     * Returns the CameraMode enum in CameraSubsystem#CameraMode
     * 
     * See enum "CameraMode" for more info
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table
     */
    public CameraMode getCameraMode() throws NTNullEntryException {
        final double mode = camMode.getDouble(RobotConst.VisionConst.ERROR);
        if (mode == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException(
                    "NetworkTable: Limelight: Camera mode (vision processing / drive) returned null");
        }
        final int modeInt = (int) Math.round(mode);
        return CameraMode.reverseLookup(modeInt);
    }

    /**
     * Returns the pipeline latency in milliseconds. Latency is the delay in
     * response.
     * 
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture
     * latency.
     *
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getPipelineLatency() throws NTNullEntryException {
        final double latency = pipelineLatency.getDouble(RobotConst.VisionConst.ERROR);
        if (latency == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Pipeline latency returned null");
        }
        return latency;
    }

    /**
     * Returns true if the snapshot mode is enabled. See #setSnapshotMode(boolean)
     * for more info
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
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

    /**
     * Returns the target's horizontal skew/rotation in degrees Skew or rotation
     * (-90 degrees to 0 degrees)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getTargetSkew() throws NTNullEntryException {
        final double skew = targetSkew.getDouble(RobotConst.VisionConst.ERROR);
        if (skew == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Skew returned null");
        }
        return skew;
    }

    /**
     * Returns the sidelength of the target's shortest side
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getTargetShort() throws NTNullEntryException {
        final double shortLength = targetShort.getDouble(RobotConst.VisionConst.ERROR);
        if (shortLength == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Short returned null");
        }
        return shortLength;
    }

    /**
     * Returns the sidelength of the target's longest side
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getTargetLong() throws NTNullEntryException {
        final double longLength = targetLong.getDouble(RobotConst.VisionConst.ERROR);
        if (longLength == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Long returned null");
        }
        return longLength;
    }

    /**
     * Returns the vertical sidelength of the rough bounding box (0 - 320 pixels)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getTargetVertical() throws NTNullEntryException {
        final double vertical = targetVertical.getDouble(RobotConst.VisionConst.ERROR);
        if (vertical == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Long returned null");
        }
        return vertical;
    }

    /**
     * Returns the horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * 
     * Will throw "NTNullEntryException" if value is not found within the network
     * table.
     */
    public double getTargetHorizontal() throws NTNullEntryException {
        final double horizontal = targetHorizontal.getDouble(RobotConst.VisionConst.ERROR);
        if (horizontal == RobotConst.VisionConst.ERROR) {
            throw new NTNullEntryException("NetworkTable: Limelight: Long returned null");
        }
        return horizontal;
    }

    // ========================================================================
    // SETTERS
    // ========================================================================

    /**
     * Setting the snap shot mode to true will take snapshots during the match.
     * 
     * 0 = stop taking snapshots 1 = take two snapshots per second
     */
    public void setSnapshotMode(final boolean enabled) {
        if (enabled) {
            snapshotMode.setDouble(1);
        } else {
            snapshotMode.setDouble(0);
        }
    }

    /**
     * Sets the LEDMode of the limelight See #LEDMode enum for more info
     */
    public void setLEDMode(final LEDMode mode) {
        final double modeNum = mode.getModeNum();
        ledMode.setValue(modeNum);
    }

    /**
     * Sets the CameraMode of the limelight See #CameraMode enum for more info
     */
    public void setCameraMode(final CameraMode mode) {
        camMode.setValue((double) mode.getModeNum());
    }

    // ========================================================================
    // ENUMS
    // ========================================================================

    /**
     * Enum for the different camera uses. VISION - used for vision
     * processing/calculating (programmer-side) DRIVER - disables processing and
     * opens stream (driver-side) See #setCameraMode(CameraMode)
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

    // ========================================================================
    // SENDABLE
    // ========================================================================

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("[Camera] Detected Target", this::hasValidTargets, null);
    
        builder.addStringProperty("[Camera] LED Mode", () -> {
            try {
                return getLEDMode().toString();
            } catch (NTNullEntryException e) {
                return "ERROR";
            }
        }, null);
        builder.addDoubleProperty("[Camera] Angle Horizontal", () -> {
            try{
                return getTargetHorizontal();
            } catch (NTNullEntryException e){
                return RobotConst.VisionConst.ERROR;
            }
        }, null);

       builder.addDoubleProperty("[Camera] Angle Vertical", () -> {
           try{
               return getTargetVertical();
           } catch (NTNullEntryException e){
               return RobotConst.VisionConst.ERROR;
           }
       }, null);

       builder.addDoubleProperty("[Camera] Target Skew", () -> {
           try{
               return getTargetSkew();
           } catch (NTNullEntryException e){
               return RobotConst.VisionConst.ERROR;
           }
       }, null);

        builder.addDoubleProperty("[Camera] Pipeline Latency", () -> {
            try{
                return getPipelineLatency();
            } catch (NTNullEntryException e){
                return RobotConst.VisionConst.ERROR;
            }
        }, null);       

 
    }

}