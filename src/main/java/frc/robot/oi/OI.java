package frc.robot.oi;

import frc.robot.constants.RobotConst.DriveConst;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap;
import frc.robot.util.Util;

public class OI{
    
    private static Joystick driver = new Joystick(JoystickMap.DRIVER_PORT);
    private static Joystick operator = new Joystick(JoystickMap.OPERATOR_PORT);


    public static Joystick getDriver(){
        return driver;
    }

    public static Joystick getOperator(){
        return operator;
    }

    /**
     * Get driver fine control
     * Right bumper
     * @return button press
     */
    public static boolean getFineControl(){
        return driver.getRawButton(JoystickMap.BUTTON_RIGHT_BUMPER);
    }

    public static double getDriverTurn(){
        return Util.setDeadzoneLimits(driver.getRawAxis(JoystickMap.RIGHT_STICK_X), DriveConst.DRIVE_TURN_TRIGGER_VALUE);
    }

    /**
     * Driver throttle stick
     * 
     * Left stick y
     * @return double from -1 to 1
     */
    public static double getDriverThrottle() {
        return Util.setDeadzoneLimits(driver.getRawAxis(JoystickMap.LEFT_STICK_Y), DriveConst.DRIVE_THORTTLE_TRIGGER_VALUE);
    }

}