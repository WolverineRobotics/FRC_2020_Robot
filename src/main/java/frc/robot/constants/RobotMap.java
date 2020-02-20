package frc.robot.constants;

public class RobotMap{

    public class Drive{
        //TODO: Change addresses

        public static final int DRIVE_LEFT_MOTOR_MASTER_ADDRESS         = -1;
        public static final int DRIVE_LEFT_MOTOR_SLAVE_ADDRESS          = -2;
        public static final int DRIVE_RIGHT_MOTOR_MASTER_ADDRESS        = -3;
        public static final int DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS         = -4;
    
    
        public static final int DRIVE_LEFT_ENCODER_A                    = -6;
        public static final int DRIVE_LEFT_ENCODER_B                    = -7;
    
        public static final int DRIVE_RIGHT_ENCODER_A                   = -4;
        public static final int DRIVE_RIGHT_ENCODER_B                   = -5;
    
        public static final int DRIVE_PIGEON_IMU_ADDRESS                = -12;
    }

    public class Sensors {
        public static final int BALL_SENSOR_1 = -1;
        public static final int BALL_SENSOR_2 = -1;
        public static final int BALL_SENSOR_3 = -1;
        public static final int BALL_SENSOR_4 = -1;
        public static final int BALL_SENSOR_5 = -1;
    }

    public class SpeedController {
        public static final int ENTRY = 20;
        public static final int CURVE = 21;
        public static final int VERTICAL_LOWER = 22;
        public static final int VERTICAL_UPPER = 23;
        public static final int SHOOT = 25;
        public static final int HOOD = 28;
    }

    public class Controller {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    public class Pneumatic {
        public static final int INTAKE_FORWARD = -1;
        public static final int INTAKE_BACKWARD = -1;
    }

}