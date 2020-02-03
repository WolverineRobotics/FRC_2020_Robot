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

    private Joystick driver;

    public DriverController(int port) {
        super(port);
        driver = super.getJoystick();
    }

    public double getThrottle() {
        return driver.getRawAxis(JoystickMap.LEFT_STICK_Y);
    }

    public double getTurn() {
        return driver.getRawAxis(JoystickMap.RIGHT_STICK_X);
    }

    public boolean getFineControl() {
        return driver.getRawButton(JoystickMap.BUTTON_RIGHT_BUMPER);
    }

}