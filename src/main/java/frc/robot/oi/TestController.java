package frc.robot.oi;

import frc.robot.constants.JoystickMap.ButtonMap;

public class TestController extends Controller {

    public TestController(int port) {
        super(port);
    }

    public boolean isPressingB() {
        return getButton(ButtonMap.BUTTON_B);
    }

}