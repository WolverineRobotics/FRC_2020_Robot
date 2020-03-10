package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConst;
import frc.robot.constants.RobotConst.VisionConst;

public class GRIPSubsystem extends SubsystemBase {

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

    public double getArea() {
        double currentLargest = 0;
        double[] array = area.getDoubleArray((double[]) null);
        if(array == null) {
            return VisionConst.ERROR;
        }
        for(double d : array) {
            if(currentLargest > d) {
                currentLargest = d;
            }
        }
        return currentLargest;
    }

    public double getCenterX(int idx) {
        double[] centerXArr = centerX.getDoubleArray((double[]) null);
        return centerXArr[0];
    }

    private void updateSDashboard() {

    }

    private boolean isArrayNull(double[] array){
        return array == null;
    }

}