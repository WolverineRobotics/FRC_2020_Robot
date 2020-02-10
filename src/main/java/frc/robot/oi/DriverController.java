package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap;

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
 * Left Trigger:
 * Right Trigger: 
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
 * POV 90:
 * POV 135:
 * POV 180:
 * POV 225:
 * POV 270:
 */
public class DriverController extends Controller {


    public DriverController(int port) {
        super(port);
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

}