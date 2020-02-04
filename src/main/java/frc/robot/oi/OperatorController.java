package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y: Drive Throttle
 * Left Stick Button:
 * 
 * Right Stick X: Drive Turn
 * Right Stick Y:
 * Right Stick Button:
 * 
 * Left Bumper:
 * Right Bumper: Drive Fine Control
 * 
 * Left Trigger:
 * Right Trigger: 
 * 
 * Button A:
 * Button B:
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

    private Joystick operator;

    public OperatorController(int port) {
        super(port);
        operator = super.getJoystick();
    }

}