package frc.robot.oi;

import frc.robot.constants.JoystickMap.AxisMap;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.constants.JoystickMap.POVMap;
import frc.robot.constants.RobotConst;

/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y: 
 * Left Stick Button:
 * 
 * Right Stick X: 
 * Right Stick Y: Shooter Hood Rotation UP=CLOSE DOWN=OPEN
 * Right Stick Button: 
 * 
 * Left Bumper: Manual Outake through BOTTOM
 * Right Bumper: Fly Wheel Run
 * 
 * Left Trigger: 
 * Right Trigger: Dumby-Proof Intake
 * 
 * Button A:
 * Button B: 
 * Button X: 
 * Button Y: 
 * 
 * Button Select: 
 * Button Start: Reset IntakeSubsystem Logic
 * 
 * POV 0: (POV Up) Outake balls from bottom
 * POV 45:
 * POV 90: 
 * POV 135:
 * POV 180: (POV Down) Run all intake motors to spit out from top.
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
        return getAxis(AxisMap.LEFT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    /**
     * @return true if right trigger is pressed down passed at least RobotConst.ControllerConst.DEADZONE_INTAKE
     */
    public boolean isHoldingRightTrigger() {
        return getAxis(AxisMap.RIGHT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    /**
     * @return true if Start is pressed
     */
    public boolean isResetIntakeLogic() {
        return getButton(ButtonMap.BUTTON_START);
    }

    public boolean isPressingB() {
        return getButton(ButtonMap.BUTTON_B);
    }

    /**
     * @return true if pressing POV North
     */
    public boolean isPOVUp() {
        return getPOV() == POVMap.POV_NORTH;
    }

    /**
     * @return true if pressing POV South
     */
    public boolean isPOVDown() {
        return getPOV() == POVMap.POV_SOUTH;
    }

    public boolean isFlyWheelRun() {
        return getButton(ButtonMap.BUTTON_RIGHT_BUMPER);
    }

    public double getHoodRotation() {
        return getAxis(AxisMap.RIGHT_STICK_Y);
    }

    public boolean isOutaking() {
        return getButton(ButtonMap.BUTTON_LEFT_BUMPER);
    }

}