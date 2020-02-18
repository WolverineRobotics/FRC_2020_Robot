package frc.robot.constants;

public class RobotConst {

    public class DriveConst {
        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.10;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.20;

        public static final double DRIVE_SPEED_REDUCTION_RATIO_FINE = 0.30;
        public static final double DRIVE_SPEED_REDUCTION_RATIO = 0.80;

        public static final double DRIVE_ENCODER_COUNTS_PER_INCH = 12.92;

        public static final double DRIVE_MAX_VOLTAGE = 10;

        public static final boolean DRIVE_SQUARE_ARCADE = false;
        public static final boolean DRIVE_SQUARE_TANK = false;

        public class CharacterizationConst {
            public static final double KS_VOLTS = 0;
            public static final double KS_VOLT_SECONDS_PER_METER = 0;
            public static final double KS_VOLT_SECONDS_SQUARED_PER_METER = 0;

            public static final double K_P_DRIVE_VELOCITY = 0;
            /**
             * The distance between the two sets of wheels on the drivetrain, in meters.
             */
            public static final double K_TRACKWIDTH_METERS = 0.7;

            public static final double K_MAX_SPEED_METERS_PER_SECOND = 0;
            public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;
            public static final double DRIVEPID_MAX_ERR_VELOCITY = 1;
            public static final double DRIVEPID_MAX_ERR_ACCELERATION = 1;

                // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double K_RAMSETE_B = 2;
            public static final double K_RAMSETE_ZETA = 0.7;
        }

    }

    public class PIDConst {
        public static final double LOCATIONCALC_DEFAULT_SETPOINT_DISTANCE = 0.8;

        public static final double GYRO_KP = 0;
        public static final double GYRO_KI = 0;
        public static final double GYRO_KD = 0;

        public static final double MAX_INTEGRAL = 0.8;

        public static final double DRIVE_PID_ERROR_TOLERANCE = 2;

        public static final double DRIVE_FF_KP = 0;
        public static final double DRIVE_FF_KI = 0;
        public static final double DRIVE_FF_KD = 0;

    }

    public class VisionConst {
        /**
         * Default value when getting NetworkTableEntry of type double. Should be a
         * value that will never appear normally, to allow for checking of this value
         * and throwing an error if nessesary.
         */
        public static final double ERROR = -99999;
        public static final double VISION_FIND_TARGETS_TIMEOUT_SECONDS = 0.5;
    }
    public class ControllerConst {
        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.10;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.20;

        public static final double DEADZONE_TRIGGER = 0.2;

        public static final double CLIMB_LEVEL_SPEED = 0.1;
        
    }
}