package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst;
import frc.robot.constants.RobotConst.VisionConst;

public class GRIPSubsystem extends SubsystemBase {

    // private final int WIDTH_PIXELS

    private NetworkTable table;

    private NetworkTableEntry area, centerX, centerY, width, height, solidity;

    public GRIPSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("grip");

        area = table.getEntry("area");
        centerX = table.getEntry("centerX");
        centerY = table.getEntry("centerY");
        width = table.getEntry("width");
        height = table.getEntry("height");
        solidity = table.getEntry("solidity");
    }

    @Override
    public void periodic() {
        updateSDashboard();
    }

    public double getLargestAreaIdx(){
        int currentLargestIndex = -1;
        double currentLargestArea = -1;
        double[] array = area.getDoubleArray((double[]) null);
        if(isArrayNull(array) || array.length == 0) {
            return -1;
        }
        for(int i = 0; i<array.length;i++ ) {
            double d = array[i];
            if(currentLargestArea > d) {
                currentLargestArea = d;
                currentLargestIndex = -1;
            }
        }
        return currentLargestIndex;
    }



    public double getArea(int idx) {
        try{
            return area.getDoubleArray((double[])null)[idx];
        }catch (Exception e) {
            return -1;
        }
    }

    public double getCenterX(int idx) {
        try{
            return centerX.getDoubleArray((double[])null)[idx];
        }catch (Exception e) {
            return -1;
        }    }


    private void updateSDashboard() {

    }

    private boolean isArrayNull(double[] array){
        return array == null;
    }

}