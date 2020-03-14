package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.MoveBallsOneStageCommand;
import frc.robot.oi.OperatorController;
import frc.robot.oi.TestController;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;
import frc.robot.subsystems.IntakeSubsystem.Ball;
import frc.robot.constants.RobotConst.IntakeConst;

public class DefaultIntakeCommand extends CommandBase {

    private final IntakeSubsystem s_intake;
    private final OperatorController oc;
    private final TestController tc;

    public DefaultIntakeCommand(final IntakeSubsystem s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
        oc = RobotContainer.getOperatorController();
        tc = RobotContainer.getTestController();
    }

    @Override
    public void initialize() {
        s_intake.setEntrySpeed(0);
        s_intake.setCurveSpeed(0);
        s_intake.setVerticalLowerSpeed(0);
        s_intake.setVerticalUpperSpeed(0);
    }

    /**
     * Dumby Proof Intake - POV DOWN
     * Outake Balls through Top (Shoot) - POV UP
     * Articulate Intake Arm - A
     * Reset Intake Logic - B Button
     * Outake Balls through Bottom - Y
     * 
     * Rev Fly Wheel - X
     * Rotate Hood - Right Stick Y UP=Forward DOWN=Reverse
     */

    @Override
    public void execute() {
        if(oc.isPOVDown()) { // if operator wants to dumby proof intake
            s_intake.setMoveBalls(true);
            // RobotContainer.getDriverController().setDriveRumble(true);
        } else if(oc.isPressingB()) { // if operator wants to auto shoot
            /**
            * Operator presses one button and the robot will:
            * 1. Rotate to vision target, will cancel command if none found
            * 2. Spin the fly wheel for X seconds
            * 3. Run all of the conveyors to unload magazine.
            */
            s_intake.mag.clear();
            s_intake.unfinishedDesto.clear();
            s_intake.ballsToRemove.clear();
            s_intake.currentPossessions.clear();
        } else if(oc.isPressingY()) { //if operator wants to outake the balls from the bottom
            s_intake.setEntrySpeed(-0.9);
            s_intake.setCurveSpeed(-0.7);
            s_intake.setVerticalLowerSpeed(-0.4);
            s_intake.setVerticalUpperSpeed(-0.3);
        } else if(oc.isPOVUp()) { //if operator wants to move balls all to fly wheel
            s_intake.setEntrySpeed(0.1);
            s_intake.setCurveSpeed(0.25);
            s_intake.setVerticalLowerSpeed(0.45);
            s_intake.setVerticalUpperSpeed(1);
        } else if(oc.isPressingA()) { //move front piston
            s_intake.toggleIntakePiston();
        // } else if(oc.isPressingX()) {
        } else {
            boolean inverted = tc.isInvert();
            if(tc.isFront()) {
                s_intake.setEntrySpeed(inverted ? -.3 : .3);
            } else if(tc.isCurve()) {
                s_intake.setCurveSpeed(inverted ? -.3 : .3);
            } else if(tc.isLowerVertical()) {
                s_intake.setVerticalLowerSpeed(inverted ? -.3 : .3);
            } else if(tc.isUpperVertical()) {
                s_intake.setVerticalUpperSpeed(inverted ? -.3 : .3);
            } else {
                s_intake.setSpeeds(0, 0, 0, 0);
                s_intake.setMoveBalls(false);
                // RobotContainer.getDriverController().setDriveRumble(false);
            }
        }
    }

    /**
     * Default commands never finish.
     * 
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}