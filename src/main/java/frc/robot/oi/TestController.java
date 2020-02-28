package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.JoystickMap.ButtonMap;

public class TestController extends Controller {

    public TestController(int port) {
        super(port);
    }

    public JoystickButton isPressingB() {
        return getButtonObject(ButtonMap.BUTTON_B);
    }

}