package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.JoystickMap.AxisMap;
import frc.robot.constants.JoystickMap.ButtonMap;
import frc.robot.constants.JoystickMap.POVMap;

public abstract class Controller {

    protected Joystick joystick;

    public Controller(int port) {
        joystick = new Joystick(port);
    }

    public boolean getButton(ButtonMap button) {
        return joystick.getRawButton(button.getPortNum());
    }

    public double getAxis(AxisMap axis) {
        return joystick.getRawAxis(axis.getPort());
    }

    public POVMap getPOV() {
        int POVInt = joystick.getPOV();
        return POVMap.getPOV(POVInt);
    }

    protected Joystick getJoystick() {
        return joystick;
    }

}