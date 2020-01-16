package frc.robot.constants;

public class RobotConst {

    public class DriveConst {

        // **********************************************************************************
        // Drive constants
        // **********************************************************************************
        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.10;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.20;

        public static final double DRIVE_SPEED_REDUCTION_RATIO_FINE = 0.30;
        public static final double DRIVE_SPEED_REDUCTION_RATIO = 0.80;

        public static final double DRIVE_ENCODER_COUNTS_PER_INCH = 12.92;

        public static final double DRIVE_MAX_VOLTAGE = 10;

        public static final boolean DRIVE_SQUARE_ARCADE = false;
        public static final boolean DRIVE_SQUARE_TANK = false;
    }

    public class PidConst {

        // **********************************************************************************
        // PID constants
        // **********************************************************************************
        public static final double LOCATIONCALC_DEFAULT_SETPOINT_DISTANCE = 0.8;

        public static final double GYRO_KP = 0;
        public static final double GYRO_KI = 0;
        public static final double GYRO_KD = 0;

    }
}