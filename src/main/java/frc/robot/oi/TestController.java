package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.constants.JoystickMap.POVMap;

public class TestController extends Controller {

    public TestController(int port) {
        super(port);
    }

    public JoystickButton isPressingB() {
        return getButtonObject(ButtonMap.BUTTON_B);
    }

    public boolean isUpperVertical() {
        return getPOV() == POVMap.POV_NORTH;
    }

    public boolean isLowerVertical() {
        return getPOV() == POVMap.POV_EAST;
    }

    public boolean isCurve() {
        return getPOV() == POVMap.POV_WEST;
    }

    public boolean isFront() {
        return getPOV() == POVMap.POV_SOUTH;
    }

    public boolean isInvert() {
        return getButton(ButtonMap.BUTTON_RIGHT_BUMPER);
    }

}