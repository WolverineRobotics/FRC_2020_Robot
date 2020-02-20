package frc.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class RobotMap{

    //TODO: Change addresses

    public static final int DRIVE_LEFT_MOTOR_MASTER_ADDRESS         = 1;
    public static final int DRIVE_LEFT_MOTOR_SLAVE_ADDRESS          = 2;
    public static final int DRIVE_RIGHT_MOTOR_MASTER_ADDRESS        = 3;
    public static final int DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS         = 4;


    public static final int DRIVE_LEFT_ENCODER_A                    = 6;
    public static final int DRIVE_LEFT_ENCODER_B                    = 7;

    public static final int DRIVE_RIGHT_ENCODER_A                   = 4;
    public static final int DRIVE_RIGHT_ENCODER_B                   = 5;

    public static final int DRIVE_PIGEON_IMU_ADDRESS                = 12;


    public static final int SHOOTER_FLYWHEEL_MOTOR_ADDRESS          = 25;
    public static final int SHOOTER_HOOD_MOTOR_ADDRESS              = 28;
    public static final int SHOOTER_HOOD_ENCODER_ADDRESS            = -1; //DIO
    public static final SerialPort.Port LIDAR_PORT = Port.kOnboard;

}