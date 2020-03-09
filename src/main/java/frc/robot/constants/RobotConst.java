package frc.robot.constants;

public class RobotConst {

    public class DriveConst {

        public static final double DRIVE_SPEED_REDUCTION_RATIO_FINE = 0.30;
        public static final double DRIVE_SPEED_REDUCTION_RATIO = 0.90;

        public static final double DRIVE_ENCODER_COUNTS_PER_METER = 521.729;

        public static final double DRIVE_MAX_VOLTAGE = 10;

        public static final boolean DRIVE_SQUARE_ARCADE = false;
        public static final boolean DRIVE_SQUARE_TANK = false;

        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.1;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.2;

        public class CharacterizationConst {
            public static final double KS_VOLTS = 0.16;
            public static final double KV_VOLT_SECONDS_PER_METER = 2.16;
            public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.319;

            public static final double K_P_DRIVE_VELOCITY = 0;
            /**
             * The distance between the two sets of wheels on the drivetrain, in meters.
             */
            public static final double K_TRACKWIDTH_METERS = 1.297;

            public static final double K_MAX_SPEED_METERS_PER_SECOND = 4;
            public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 15;
            public static final double DRIVEPID_MAX_ERR_VELOCITY = 0.06;
            public static final double DRIVEPID_MAX_ERR_ACCELERATION = 0.2;

            public static final double K_MAX_TURN_DEG_PER_SECOND = 80;
            public static final double K_MAX_TURN_ACCEL_DEG_PER_SECOND_SQUARED = 30;

                // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double K_RAMSETE_B = 2;
            public static final double K_RAMSETE_ZETA = 0.7;
        }

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

        public static final double MAX_INTEGRAL = 0.8;

        public static final double DRIVE_PID_ERROR_TOLERANCE = 2;

        public static final double DRIVE_FF_KP = 0;
        public static final double DRIVE_FF_KI = 0;
        public static final double DRIVE_FF_KD = 0;

        public static final double DRIVE_STRAIGHT_KP = 0.001;
        public static final double DRIVE_STRAIGHT_KI = 0.00004;
        public static final double DRIVE_STRAIGHT_KD = 0.005;

        public static final double DRIVE_TURN_KP = 0.016;
        public static final double DRIVE_TURN_KI = 0.00005;
        public static final double DRIVE_TURN_KD = 0.00001;

        public static final double DRIVE_TURN_TOLERANCE_DEG = 1.5;
        public static final double DRIVE_TURN_TOLERANCE_DEG_PER_SECOND = 1.4;

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

    public class ShooterConst{

        // **********************************************************************************
        // Shooter constants
        // **********************************************************************************

        public static final double SHOOTER_FLYWHEEL_GEAR_RATIO = 22/18;
        public static final int HOOD_ENCODER_ZERO_POSITION = 0;

        public static final double HOOD_MOTOR_POWER = 0.4;

        public static final double SHOOTER_SPEED = 1;
    }

    public class ControllerConst {
        public static final double DRIVE_TURN_TRIGGER_VALUE = 0.10;
        public static final double DRIVE_THORTTLE_TRIGGER_VALUE = 0.20;

        public static final double DEADZONE_TRIGGER = 0.2;
    }
}
