package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;

public class RevAbsoluteEncoder extends DutyCycle {

    public RevAbsoluteEncoder(int DIOPort) {
        this(new DigitalInput(DIOPort));
    }

    public RevAbsoluteEncoder(DigitalSource digitalSource) {
        super(digitalSource);
    }

    /**
     * Returns -1 if noting is connected.
     * @return
     */
    public int getEncoderPosition() {
        double encoderTick = getOutput();
        encoderTick = Math.round(encoderTick * 1025) - 1;
        return (int) encoderTick;
    }

    public boolean isConnected(){
        return getEncoderPosition() == -1;
    }

}