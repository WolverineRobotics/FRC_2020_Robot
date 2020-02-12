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
 * Right Stick Y: [up] = hood down | [down] = hood up
 * Right Stick Button:
 * 
 * Left Bumper:
 * Right Bumper:
 * 
 * Left Trigger: Intake Balls
 * Right Trigger: Outake Balls
 * 
 * Button A:
 * Button B: Auto-Shoot (See DefaultIntakeCommand)
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
public class OperatorController extends Controller {

    public OperatorController(int port) {
        super(port);
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
     * @return double value from -1 to 1.
     */
    public double getRightStickY() {
        return operator.getRawAxis(JoystickMap.RIGHT_STICK_Y);
    }

    /**
     * @return true if B is pressed
     */
    public boolean getAutoShootButton() {
        return operator.getRawButton(JoystickMap.BUTTON_B);
    }

}