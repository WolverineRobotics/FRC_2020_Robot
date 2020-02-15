package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.util.UARTLidar;

public class LidarSubsystem extends SubsystemBase{

    private final UARTLidar lidar;

    public LidarSubsystem(){
        this(RobotMap.LIDAR_PORT);
    }

    public LidarSubsystem(SerialPort.Port port){
        super();
        lidar = new UARTLidar(port);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public int getDistanceCM(){
        return lidar.getDistanceCM();
    }

    public boolean isMeasurementAccurate(){
        return lidar.isMeasurementAccurate();
    }

    public boolean isMeasurementValid(){
        return lidar.isMeasurementValid();
    }

    @Override
    public void periodic() {
        lidar.recordDistance();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Lidar Subsystem");
        builder.addDoubleProperty("[Lidar] Distance cm", this::getDistanceCM, null);
    }
    
}