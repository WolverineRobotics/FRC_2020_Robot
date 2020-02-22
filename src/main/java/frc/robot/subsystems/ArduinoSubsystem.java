package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase{

    private SerialPort arduino;

    private Timer timer;
    private int distanceCm;

    public ArduinoSubsystem(){
        try {
            arduino = new SerialPort(9600, Port.kUSB);
            System.out.println("Connected arduino to kUSB");
        } catch (Exception e0) {
            try {
                arduino = new SerialPort(9600, Port.kUSB1);
                System.out.println("Connected arduino to kUSB1");
            } catch (Exception e1) {
                try {
                    arduino = new SerialPort(9600, Port.kUSB2);
                    System.out.println("Connected arduino to kUSB2");
                } catch (Exception e2) {
                    System.out.println("Failed all USB connection attempts");
                }
            }
        }
        timer = new Timer();
        timer.start();
        distanceCm = -1;
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
        if(arduino.getBytesReceived() > 0) {
            System.out.println(arduino.readString());
            //PARSE STRING TO INT
            int amountBytes = arduino.getBytesReceived();
            byte[] read = arduino.read(amountBytes);
            
            String parse = "";
            for(byte b : read) {
                parse += b;
            }

            //PASS TO VARIABLE
            try {
                distanceCm = Integer.parseInt(parse);
            } catch (NumberFormatException e) {
                System.out.println("Error parsing Arduino bytes: " + read);
            }
        }
        updateDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // super.initSendable(builder);
        // builder.setSmartDashboardType("Arduino Subsystem");
        // builder.addDoubleProperty("[Arduino] Distance Cm", this::getDistanceCM, null);
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[Arduino] Lidar Distance Cm", distanceCm);
    }
    
}