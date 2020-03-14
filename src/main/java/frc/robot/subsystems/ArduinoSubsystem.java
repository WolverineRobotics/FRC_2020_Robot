package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {

    private SerialPort serial;

    private int distanceCm = -1;
    private int lastValidDistanceCm = -1;
    private int invalidMeasurementCounter;

    public ArduinoSubsystem() {
        super();
        try {
            serial = new SerialPort(115200, Port.kUSB);
            System.out.println("Connected arduino to kUSB");
        } catch (Exception e0) {
            try {
                serial = new SerialPort(115200, Port.kUSB1);
                System.out.println("Connected arduino to kUSB1");
            } catch (Exception e1) {
                try {
                    serial = new SerialPort(115200, Port.kUSB2);
                    System.out.println("Connected arduino to kUSB2");
                } catch (Exception e2) {
                    System.out.println("Failed all USB connection attempts");
                }
            }
        }
        distanceCm = -1;
    }

    /**
     * Get the Distance of the Lidar in Centimetres
     * 
     * @return default: -1. Lidar Sensor distance in centimetres
     */
    public int getDistanceCM() {
        return distanceCm;
    }


    public int getLastValidDistanceCm() {
        return lastValidDistanceCm;
    }

    public int getInvalidMeasurementCount() {
        return invalidMeasurementCounter;
    }

    public double getDistanceIN(){
        double distanceCM = getDistanceCM();
        if(distanceCM == -1){
            return -1;
        }
        return Units.metersToInches(distanceCM/100);
    }

    @Override
    public void periodic() {
        measureDistance();

        if (distanceCm != -1) {
            lastValidDistanceCm = distanceCm;
            invalidMeasurementCounter = 0;
        } else {
            invalidMeasurementCounter++;
        }

        updateDashboard();
    }

    private void measureDistance() {
        if (serial != null && serial.getBytesReceived() > 0) {

            String strOutput = serial.readString();

            if (strOutput == null || strOutput.length() == 0) {
                return;
            }

            String[] strArr = strOutput.split("\n");
            if (strArr.length < 1) {
                return;
            }

            if (strArr[strArr.length - 1].endsWith("\n")) {
                distanceCm = Integer.parseInt(strArr[strArr.length - 1].trim());
            } else {
                try {
                    distanceCm = Integer.parseInt(strArr[strArr.length - 2].trim());
                } catch (ArrayIndexOutOfBoundsException e) {
                    // System.out.println(
                    // "[Arduino] Distance string array doesn't end with newline but only has 1
                    // element, string: "
                    // + strOutput);
                    try {
                        distanceCm = Integer.parseInt(strArr[0].trim());
                        // System.out.println(distanceCm);
                    } catch (Exception e2) {
                        distanceCm = -1;
                    }
                }
            }

            // System.out.println(distanceCm);

        }
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // super.initSendable(builder);
        // builder.setSmartDashboardType("Arduino Subsystem");
        // builder.addDoubleProperty("[Arduino] Distance Cm", this::getDistanceCM,
        // null);
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[Arduino] Lidar Distance Cm", distanceCm);
        SmartDashboard.putNumber("[Arduino] Lidar Distance In", getDistanceIN());

        // SmartDashboard.putString("[Arduino] Raw", raw);
    }

}