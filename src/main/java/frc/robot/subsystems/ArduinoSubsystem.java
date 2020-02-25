package frc.robot.subsystems;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase{

    private SerialPort serial;

    private String raw;
    private int distanceCm;

    public ArduinoSubsystem(){
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
        raw = "nothing";
    }

    /**
     * Get the Distance of the Lidar in Centimetres
     * @return default: -1. Lidar Sensor distance in centimetres
     */
    public int getDistanceCM(){
        return distanceCm;
    }

    @Override
    public void periodic() {
        if(serial != null) {
            String read = serial.readString();
            byte[] ascii = read.getBytes(StandardCharsets.US_ASCII);
            String asciiString = Arrays.toString(ascii);
            raw = read;
            if(read.length() > 0) {
                int distance;
                try {
                    distance = Integer.parseInt(asciiString);
                    distanceCm = distance;
                } catch (Exception e) {
                    System.out.println("Could not parse String: " + asciiString);
                }
            }
            updateDashboard();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // super.initSendable(builder);
        // builder.setSmartDashboardType("Arduino Subsystem");
        // builder.addDoubleProperty("[Arduino] Distance Cm", this::getDistanceCM, null);
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[Arduino] Lidar Distance Cm", distanceCm);
        SmartDashboard.putString("[Arduino] Raw", raw);
    }
    
}