package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap;
import frc.robot.constants.RobotConst;
import frc.robot.constants.JoystickMap.AxisMap;

/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y: Drive Throttle
 * Left Stick Button:
 * 
 * Right Stick X: Drive Turn
 * Right Stick Y:
 * Right Stick Button:
 * 
 * Left Bumper:
 * Right Bumper: Drive Fine Control
 * 
 * Left Trigger: Climb DOWN
 * Right Trigger: Climb UP
 * 
 * Button A:
 * Button B:
 * Button X:
 * Button Y:
 * 
 * Button Select:
 * Button Start:
 * 
 * POV 0: 
 * POV 45:
 * POV 90: Climb LEVEL Right
 * POV 135:
 * POV 180: Climb LEVEL Left
 * POV 225:
 * POV 270:
 */
public class DriverController extends Controller {

    private Joystick driver;

    public DriverController(int port) {
        super(port);
        driver = super.getJoystick();
    }

    public double getThrottle() {
        return getAxis(JoystickMap.AxisMap.LEFT_STICK_Y);
    }

    public double getLeftThrottle() {
        return getAxis(JoystickMap.AxisMap.LEFT_STICK_Y);
    }

    public double getTurn() {
        return getAxis(JoystickMap.AxisMap.RIGHT_STICK_X);
    }

    public double getRightThrottle(){
        return getAxis(AxisMap.RIGHT_STICK_Y);
    }

    public boolean getFineControl() {
        return getButton(JoystickMap.ButtonMap.BUTTON_RIGHT_BUMPER);
    }

    public double getRightTrigger() {
        double value = driver.getRawAxis(JoystickMap.RIGHT_TRIGGER);
        if(value > RobotConst.ControllerConst.DEADZONE_TRIGGER) {
            return value;
        }
        return 0;
    }

    public double getLeftTrigger() {
        double value = driver.getRawAxis(JoystickMap.LEFT_TRIGGER);
        if(value > RobotConst.ControllerConst.DEADZONE_TRIGGER) {
            return value;
        }
        return 0;
    }

    public boolean isPOVRight() {
        return driver.getPOV() == JoystickMap.POV_EAST;
    }

    public boolean isPOVLeft() {
        return driver.getPOV() == JoystickMap.POV_WEST;
    }

}