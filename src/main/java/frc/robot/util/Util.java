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

}