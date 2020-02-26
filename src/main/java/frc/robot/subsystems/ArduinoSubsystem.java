package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase{

    private SerialPort serial;

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
        if(serial != null && serial.getBytesReceived() > 0) {
            // String read = serial.readString();
            // byte[] ascii = read.getBytes(StandardCharsets.US_ASCII);
            // String asciiString = Arrays.toString(ascii);
            // raw = read;
            // byte[] read = serial.read(5);
            // if(read[0] != 0xF0){
            //     boolean arrChanged = false;
            //     for(int i = 0; i < read.length; i++){
            //         if(read[i] == 0xF0){
            //             read = changeStart(read, i);
            //             arrChanged = true;
            //             break;
            //         }
            //     }
            //     if(!arrChanged || read == null){
            //         distanceCm = -1;
            //         return;
            //     }
            // }

            // int distance = Byte.toUnsignedInt(read[4]) + (Byte.toUnsignedInt(read[3]) << 8);
            // this.distanceCm = distance;

            String strOutput = serial.readString();
            
            if(strOutput == null || strOutput.length() == 0){
                return;
            }

            String[] strArr = strOutput.split("\n");
            if (strArr.length < 2){
                return;
            }

            if (strArr[strArr.length - 1].endsWith("\n")){
                distanceCm = Integer.parseInt(strArr[strArr.length - 1].trim());
            }else{
                distanceCm = Integer.parseInt(strArr[strArr.length - 2].trim());
            }
            
            updateDashboard();
        }
    }

    private byte[] changeStart(byte[] oldArr, int offset){
        int numToRead = oldArr.length - offset - 1;
        if(serial == null || serial.getBytesReceived() < numToRead){
            return null;
        }
        byte[] newArr = serial.read(numToRead);
        
        if(newArr.length != numToRead){
            return null;
        }

        // Byte[] oldArr2 = oldArr;

        ArrayList<Byte> list = new ArrayList<>();
        for(int i = offset; i < oldArr.length; i++) {
            list.add(oldArr[i]);
        }
        for(byte b: newArr){
            list.add(b);
        }
        Byte[] objArr = list.toArray(new Byte[0]);

        byte[] retArr = new byte[objArr.length];

        for(int i = 0; i < objArr.length; i++){
            retArr[i] = (byte) objArr[i];
        }

        return retArr;
        // Collections.addAll(oldArr, list);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // super.initSendable(builder);
        // builder.setSmartDashboardType("Arduino Subsystem");
        // builder.addDoubleProperty("[Arduino] Distance Cm", this::getDistanceCM, null);
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[Arduino] Lidar Distance Cm", distanceCm);
        // SmartDashboard.putString("[Arduino] Raw", raw);
    }
    
}