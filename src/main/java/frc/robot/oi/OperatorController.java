package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap;
import frc.robot.constants.RobotConst;

/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y: 
 * Left Stick Button:
 * 
 * Right Stick X: 
 * Right Stick Y:
 * Right Stick Button:
 * 
 * Left Bumper:
 * Right Bumper:
 * 
 * Left Trigger: Intake Balls - programmed with sensors / should be idiot proof
 * Right Trigger: Outake Balls from top
 * 
 * Button A:
 * Button B: Auto-Shoot (See DefaultIntakeCommand)
 * Button X: 
 * Button Y: 
 * 
 * Button Select:
 * Button Start:
 * 
 * POV 0: (POV Up) Outake balls from bottom
 * POV 45:
 * POV 90: 
 * POV 135:
 * POV 180: (POV Down) Entry intake motor only
 * POV 225:
 * POV 270:
 */
public class OperatorController extends Controller {

    private Joystick operator;

    public OperatorController(int port) {
        super(port);
        operator = super.getJoystick();
    }

    /**
     * @return true if left trigger is pressed down passed at least RobotConst.ControllerConst.DEADZONE_INTAKE
     */
    public boolean isHoldingLeftTrigger() {
        return operator.getRawAxis(JoystickMap.LEFT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_INTAKE;
    }

    /**
     * @return true if right trigger is pressed down passed at least RobotConst.ControllerConst.DEADZONE_INTAKE
     */
    public boolean isHoldingRightTrigger() {
        return operator.getRawAxis(JoystickMap.RIGHT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_INTAKE;
    }

    /**
     * @return true if B is pressed
     */
    public boolean getAutoShootButton() {
        return operator.getRawButton(JoystickMap.BUTTON_B);
    }

    /**
     * @return true if pressing POV North
     */
    public boolean isPOVUp() {
        return operator.getPOV() == JoystickMap.POV_NORTH;
    }

    /**
     * @return true if pressing POV South
     */
    public boolean isPOVDown() {
        return operator.getPOV() == JoystickMap.POV_SOUTH;
    }

}