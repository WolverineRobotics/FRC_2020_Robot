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
 * Right Stick Y: Shooter Hood Rotation UP=CLOSE DOWN=OPEN
 * Right Stick Button: 
 * 
 * Left Bumper: 
 * Right Bumper:
 * 
 * Left Trigger: 
 * Right Trigger: Rev Fly Wheel
 * 
 * Button A: Articulate Arm
 * Button B: Reset Intake Logic
 * Button X: 
 * Button Y: Outake Balls through Bottom
 * 
 * Button Select: 
 * Button Start:
 * 
 * POV 0: (POV Up) Move balls UP
 * POV 45:
 * POV 90: 
 * POV 135:
 * POV 180: (POV Down) Dumby proof in-intake
 * POV 225:
 * POV 270:
 * 
 * 
 */
public class OperatorController extends Controller {

    private Joystick operator;

    public OperatorController(int port) {
        super(port);
        operator = super.getJoystick();
    }

    public double getRightStickY() {
        return operator.getRawAxis(JoystickMap.RIGHT_STICK_Y);
    }

    public double getRightTrigger() {
        return operator.getRawAxis(JoystickMap.RIGHT_TRIGGER);
    }

    public boolean isPressingA() {
        return operator.getRawButton(JoystickMap.BUTTON_A);
    }

    public boolean isPressingB() {
        return operator.getRawButton(JoystickMap.BUTTON_B);
    }

    public boolean isPressingY() {
        return operator.getRawButton(JoystickMap.BUTTON_Y);
    }

    public boolean isPOVUp() {
        return operator.getPOV() == JoystickMap.POV_NORTH;
    }

    public boolean isPOVDown() {
        return operator.getPOV() == JoystickMap.POV_SOUTH;
    }

}