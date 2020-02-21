package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;

public class RevAbsoluteEncoder {

    private DutyCycle absEncoder;

    public final int COUNTS_PER_REVOLUTION = 1024;
    private int zeroPosition;

    public RevAbsoluteEncoder(int DIOPort) {
        this(new DigitalInput(DIOPort));
    }

    public RevAbsoluteEncoder(int DIOPort, int zeroPosition) {
        this(new DigitalInput(DIOPort), zeroPosition);
    }

    public RevAbsoluteEncoder(DigitalSource digitalSource) {
        this(digitalSource, 0);
    }

    public RevAbsoluteEncoder(DigitalSource digitalSource, int zeroPosition) {
        absEncoder = new DutyCycle(digitalSource);
        this.zeroPosition = zeroPosition;
    }

    public void setZeroPosition(int zeroPosition) {
        this.zeroPosition = zeroPosition;
    }

    public int getZeroPosition() {
        return zeroPosition;
    }

    private int subtractZeroPosition(int encoderTicks, int zeroPosition) {
        encoderTicks = encoderTicks - zeroPosition;
        encoderTicks = Math.floorMod(encoderTicks, COUNTS_PER_REVOLUTION);
        return encoderTicks;
    }

    /**
     * Returns the encoder position taking into account the zero position, or -1 if
     * noting is connected.
     * 
     * @return
     */
    public int getEncoderPosition() {
        int encoderTicks = getRawEncoderPosition();
        if (encoderTicks == -1) {
            return encoderTicks;
        }
        encoderTicks = subtractZeroPosition(encoderTicks, zeroPosition);
        return encoderTicks;
    }

    /**
     * Returns the raw encoder position, or -1 if noting is connected.
     * 
     * @return
     */
    public int getRawEncoderPosition() {
        double dutyCycleRatio = absEncoder.getOutput();
        int encoderTicks = (int) Math.round(dutyCycleRatio * (COUNTS_PER_REVOLUTION + 1)) - 1;
        return encoderTicks;
    }

    public boolean isConnected() {
        return !(getEncoderPosition() == -1);
    }

}