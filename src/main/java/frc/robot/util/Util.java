package frc.robot.util;

import edu.wpi.first.wpiutil.math.MathUtil;

public class Util {

    // Allows for input values less than trigger values.
    // Piecewise function, if less than deadzone value, 0
    // Else linear
    // https://www.desmos.com/calculator/h7gnlk21iz

    public static double getDeadzoneResult(double controlInput, double deadzoneAmount) {

        if (Math.abs(controlInput) < deadzoneAmount) {
            controlInput = 0;
        } else {
            if (controlInput > 0) {
                controlInput = ((controlInput - deadzoneAmount) / (1 - deadzoneAmount));
            } else {
                controlInput = ((controlInput + deadzoneAmount) / (1 - deadzoneAmount));
            }
        }
        return controlInput;
    }

    // Ensures motor power values are between -1 and 1
    public static double getMotorLimits(double motorPower) {
        return MathUtil.clamp(motorPower, -1, 1);
    }

    public static double setDeadzoneLimits(double controlInput, double deadzoneValue) {

        controlInput = getDeadzoneResult(controlInput, deadzoneValue);
        controlInput = getMotorLimits(controlInput);
        return controlInput;

    }

    // Ensures that a double is between 0 and 1
    // If less than 0, will be set to 0
    // If greater than 1, will be set to one
    public static double zeroToOne(double num) {
        return MathUtil.clamp(num, 0, 1);
    }

    public static double toVoltage(double speed, double maxVoltage) {
        return (speed * maxVoltage);
    }

    public static double normalizeValue(double value, double min, double max) {
        return Math.IEEEremainder(value - min, max - min) + min;
    }

    public static double addGyroValues(double value1, double value2) {
        final double MIN_GYRO_VALUE = 0;
        final double MAX_GYRO_VALUE = 360;
        double val = value1 + value2;
        return normalizeValue(val, MIN_GYRO_VALUE, MAX_GYRO_VALUE);
    }

    public static double subtractGyroValues(double value1, double value2) {
        final double MIN_GYRO_VALUE = 0;
        final double MAX_GYRO_VALUE = 360;
        final double INPUT_RANGE = MAX_GYRO_VALUE - MIN_GYRO_VALUE;
        double error = value1 - value2;

        error %= INPUT_RANGE;
        if (Math.abs(error) > INPUT_RANGE / 2) {
            if (error > 0) {
                return error - INPUT_RANGE;
            } else {
                return error + INPUT_RANGE;
            }
        }
        return error;
    }

    /**
     * Wraps error around, returning the shortest path for a continuous error.
     */
    public static double getContinuousError(double error, double inputRange) {
        if (inputRange > 0) {
          error %= inputRange;
          if (Math.abs(error) > inputRange / 2) {
            if (error > 0) {
              return error - inputRange;
            } else {
              return error + inputRange;
            }
          }
        }
        return error;
      }

    public static long getSeconds(long milliseconds) {
        return (milliseconds * 1000);
    }

}