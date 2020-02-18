package frc.robot.constants;

public class RobotConst {

    public class DriveConst {

        public static final double DRIVE_SPEED_REDUCTION_RATIO_FINE = 0.30;
        public static final double DRIVE_SPEED_REDUCTION_RATIO = 0.80;

        public static final double DRIVE_ENCODER_COUNTS_PER_INCH = 12.92;

        public static final double DRIVE_MAX_VOLTAGE = 10;

        public static final boolean DRIVE_SQUARE_ARCADE = false;
        public static final boolean DRIVE_SQUARE_TANK = false;

    }

    public class IntakeConst {
        public static final int ENTRY_SPEED_REDUCTION_RATIO = 1;
        public static final int CURVE_SPEED_REDUCTION_RATIO = 1;
        public static final int VERTICAL_SPEED_REDUCTION_RATIO = 1;
        public static final int LOWER_VERTICAL_SPEED_REDUCTION_RATIO = 1;
        public static final int UPPER_VERTICAL_SPEED_REDUCTION_RATIO = 1;

        public static final double ENTRY_SPEED = 0.3;
        public static final double CURVE_SPEED = 0.5;
        public static final double LOWER_VERTICAL_SPEED = 0.5;
        public static final double UPPER_VERTICAL_SPEED = 0.5;
        public static final double UPPER_VERTICAL_SHOOT_SPEED = 0.5;
    }

    public class PIDConst {
        public static final double LOCATIONCALC_DEFAULT_SETPOINT_DISTANCE = 0.8;

        public static final double GYRO_KP = 0;
        public static final double GYRO_KI = 0;
        public static final double GYRO_KD = 0;
    }

    public class VisionConst {
        /**
         * Default value when getting NetworkTableEntry of type double. Should be a
         * value that will never appear normally, to allow for checking of this value
         * and throwing an error if nessesary.
         */
        public static final double ERROR = -99999;
    }

    public class ControllerConst {
        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.10;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.20;

        public static final double DEADZONE_TRIGGER = 0.2;
    }


}
