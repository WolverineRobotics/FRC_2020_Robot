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
 * Left Trigger: Climb down
 * Right Trigger:  Climb up
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

    public boolean isClimbDown(){   
    return driver.getRawAxis(JoystickMap.LEFT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    public boolean isClimbUp(){
        return driver.getRawAxis(JoystickMap.RIGHT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    public boolean isRight() {
        return driver.getPOV() == JoystickMap.POV_EAST;
    }

    public boolean isLeft(){
        return driver.getPOV() == JoystickMap.POV_WEST;
    }
    

}