package frc.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class RobotMap{

    public class Drive{
        // //TODO: Change addresses

        // public static final int DRIVE_LEFT_MOTOR_MASTER_ADDRESS         = -1;
        // public static final int DRIVE_LEFT_MOTOR_SLAVE_ADDRESS          = -2;
        // public static final int DRIVE_RIGHT_MOTOR_MASTER_ADDRESS        = -3;
        // public static final int DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS         = -4;
    
    
        public static final int DRIVE_LEFT_ENCODER_A                    = 10;
        public static final int DRIVE_LEFT_ENCODER_B                    = 11;
    
        public static final int DRIVE_RIGHT_ENCODER_A                   = 12;
        public static final int DRIVE_RIGHT_ENCODER_B                   = 13;
    
    }

    public class Sensors {
        public static final int BALL_SENSOR_6 = 7;
        public static final int BALL_SENSOR_1 = 5;
        public static final int BALL_SENSOR_2 = 4;
        public static final int BALL_SENSOR_3 = 3;
        public static final int BALL_SENSOR_4 = 2;
        public static final int BALL_SENSOR_5 = 1;
    }

    public class SpeedController {
        public static final int ENTRY = 20;
        public static final int CURVE = 21;
        public static final int VERTICAL_LOWER = 22;
        public static final int VERTICAL_UPPER = 23;
        public static final int SHOOT = 25;
        public static final int HOOD = 28;
        public static final int CLIMB = 30; //neo
        public static final int CLIMB_LEVEL = 33; //talon
    }

    public class Controller {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
        public static final int TEST = 2;
    }

    public class Pneumatic {
        public static final int PCM = 3;

        public static final int INTAKE_FORWARD = 2;
        public static final int INTAKE_BACKWARD = 3;

        public static final int CLIMB_LOCK_FORWARD = 4;
        public static final int CLIMB_LOCK_REVERSE = 5;
    }
    //TODO: Change addresses

    public static final int DRIVE_LEFT_MOTOR_MASTER_ADDRESS         = 10;
    public static final int DRIVE_LEFT_MOTOR_SLAVE_ADDRESS_1         = 11;
    public static final int DRIVE_LEFT_MOTOR_SLAVE_ADDRESS_2         = 12;

    public static final int DRIVE_RIGHT_MOTOR_MASTER_ADDRESS        = 15;
    public static final int DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS_1        = 16;
    public static final int DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS_2        = 17;

    public static final int DRIVE_PIGEON_IMU_ADDRESS                = 6;


    public static final int SHOOTER_FLYWHEEL_MOTOR_ADDRESS          = 25;
    public static final int SHOOTER_HOOD_MOTOR_ADDRESS              = 28;
    public static final int SHOOTER_HOOD_ENCODER_ADDRESS            = 0; //DIO
    public static final SerialPort.Port LIDAR_PORT = Port.kMXP;

    public static final int CLIMB_ENCODER = 6;

}