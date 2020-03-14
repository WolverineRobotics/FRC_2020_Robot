package frc.robot.oi;

import frc.robot.constants.JoystickMap.AxisMap;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.constants.JoystickMap.POVMap;
import frc.robot.constants.JoystickMap;
import frc.robot.constants.RobotConst;

/**
 * Controller Map:
 * 
 * Left Stick X: Left Stick Y: Left Stick Button:
 * 
 * Right Stick X: Right Stick Y: Shooter Hood Rotation UP=CLOSE DOWN=OPEN Right
 * Stick Button:
 * 
 * Left Bumper: Right Bumper:
 * 
 * Left Trigger: Right Trigger: Rev Fly Wheel
 * 
 * Button A: Articulate Arm Button B: Reset Intake Logic Button X: Button Y:
 * Outake Balls through Bottom
 * 
 * Button Select: Button Start:
 * 
 * POV 0: (POV Up) Move balls UP POV 45: POV 90: POV 135: POV 180: (POV Down)
 * Dumby proof in-intake POV 225: POV 270:
 * 
 * 
 */
public class OperatorController extends Controller {

    public OperatorController(int port) {
        super(port);
    }

    public double getRightStickY() {
        return getAxis(AxisMap.RIGHT_STICK_Y);
    }

    /**
     * @return true if left trigger is pressed down passed at least
     *         RobotConst.ControllerConst.DEADZONE_INTAKE
     */
    public boolean isHoldingLeftTrigger() {
        return getAxis(AxisMap.LEFT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    public double getRightTrigger() {
        return getAxis(AxisMap.RIGHT_TRIGGER);
    }

    /**
     * @return true if right trigger is pressed down passed at least
     *         RobotConst.ControllerConst.DEADZONE_INTAKE
     */
    public boolean isHoldingRightTrigger() {
        return getAxis(AxisMap.RIGHT_TRIGGER) > RobotConst.ControllerConst.DEADZONE_TRIGGER;
    }

    public boolean isPressingA() {
        return getButtonPressed(ButtonMap.BUTTON_A);
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

    public boolean isPressingY() {
        return getButton(ButtonMap.BUTTON_Y);
    }

    public boolean isPOVUp() {
        POVMap currentPov = getPOV();
        return (currentPov == POVMap.POV_NORTH || currentPov == POVMap.POV_NORTH_EAST
                || currentPov == POVMap.POV_NORTH_WEST);
    }

    public boolean isPOVDown() { 
        return getPOV() == POVMap.POV_SOUTH;
    }

    public boolean isPressingX() {
        return getButtonPressed(ButtonMap.BUTTON_X);
    }

    public boolean isFlyWheelRun() {
        return getButton(ButtonMap.BUTTON_RIGHT_BUMPER);
    }

    public double getHoodRotation() {
        return getAxis(AxisMap.RIGHT_STICK_Y);
    }

    public boolean isFlywheelSetVoltage() {
        return getButton(ButtonMap.BUTTON_LEFT_BUMPER);
        // return false;
    }

    public boolean isIntakeShootTopStageOnly(){
        return getPOV() == POVMap.POV_WEST;
    }

    public boolean isShooter10V(){
        return Math.abs(getAxis(AxisMap.LEFT_TRIGGER)) > 0.3;
    }

}