package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap;
import frc.robot.constants.RobotConst;

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
 * Left Trigger: Climb Level FORWARD
 * Right Trigger: Climb level BACKWARD
 * 
 * Button A:
 * Button B:
 * Button X:
 * Button Y:
 * 
 * Button Select:
 * Button Start:
 * 
 * POV 0: (POV UP) Climb Up
 * POV 45:
 * POV 90:
 * POV 135:
 * POV 180: (POV DOWN) Climb Down
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
        return getAxis(JoystickMap.AxisMap.LEFT_STICK_X);
    }

    public double getTurn() {
        return getAxis(JoystickMap.AxisMap.RIGHT_STICK_X);
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

    public boolean isPOVUp() {
        return driver.getPOV() == JoystickMap.POV_NORTH;
    }

    public boolean isPOVDown() {
        return driver.getPOV() == JoystickMap.POV_SOUTH;
    }

}