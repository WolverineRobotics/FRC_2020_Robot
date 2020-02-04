package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;

public abstract class Controller {

    private Joystick joystick;

    public Controller(int port) {
        joystick = new Joystick(port);
    }

    protected Joystick getJoystick() {
        return joystick;
    }

}