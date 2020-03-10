package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.JoystickMap;
import frc.robot.constants.JoystickMap.AxisMap;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.constants.JoystickMap.POVMap;
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
 * Left Trigger: Climb DOWN
 * Right Trigger: Climb UP
 * 
 * Button A:
 * Button B: Climb Lock Toggle
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


    public DriverController(int port) {
        super(port);
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
        double value = getAxis(AxisMap.RIGHT_TRIGGER);
        if(value > RobotConst.ControllerConst.DEADZONE_TRIGGER) {
            return value;
        }
        return 0;
    }

    public double getLeftTrigger() {
        double value = getAxis(AxisMap.LEFT_TRIGGER);
        if(value > RobotConst.ControllerConst.DEADZONE_TRIGGER) {
            return value;
        }
        return 0;
    }

    public boolean isPOVLeft(){
        return getPOV() == POVMap.POV_WEST;
    }
    
    public boolean isPOVRight(){
        return getPOV() == POVMap.POV_EAST;
    }

    public JoystickButton getRotateVisionTargetButtonObj(){
        return getButtonObject(ButtonMap.BUTTON_A);
    }

    public boolean isPressingB(){
        return getButton(JoystickMap.ButtonMap.BUTTON_B);
    }

    public boolean isPressingY() {
        return getButton(ButtonMap.BUTTON_Y);
    }

    public boolean isLimelightLed(){
        return getButton(ButtonMap.BUTTON_LEFT_BUMPER);
    }

    public void setDriveRumble(boolean toRumble) {
        super.getJoystick().setRumble(RumbleType.kLeftRumble, 1);
        super.getJoystick().setRumble(RumbleType.kRightRumble, 1);
    }

}