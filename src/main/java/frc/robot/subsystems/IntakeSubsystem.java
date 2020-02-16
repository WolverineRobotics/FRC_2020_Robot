package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private DigitalInput sensor1; //entry sensor of the intake
    private DigitalInput sensor2; //sensor located at curve
    private DigitalInput sensor3; //sensor located at bottom of vertical conveyor
    private DigitalInput sensor4; //sensor located at middle of vertical conveyor
    private DigitalInput sensor5; //sensor located at top of vertical conveyor

    private CANSparkMax entry; // main intake, entry point of ball
    private CANSparkMax curve; // curves ball into vertical conveyor
    private CANSparkMax verticalLower; // controls vertical conveyor lower
    private CANSparkMax verticalUpper; //controls vertical conveyor upper 

    private DoubleSolenoid piston; //piston that opens the front intake. Forward=Intake Openened and Reverse=Intake Closed

    // initializes all of the components within the subsystem
    public IntakeSubsystem() {
        sensor1 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_1);
        sensor2 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_2);
        sensor3 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_3);
        sensor4 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_4);
        sensor5 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_5);

        entry = new CANSparkMax(RobotMap.SpeedController.ENTRY, MotorType.kBrushless);
        curve = new CANSparkMax(RobotMap.SpeedController.CURVE, MotorType.kBrushless);
        verticalLower = new CANSparkMax(RobotMap.SpeedController.VERTICAL_LOWER, MotorType.kBrushless);
        verticalUpper = new CANSparkMax(RobotMap.SpeedController.VERTICAL_UPPER, MotorType.kBrushless);

        piston = new DoubleSolenoid(RobotMap.Pneumatic.INTAKE_FORWARD, RobotMap.Pneumatic.INTAKE_BACKWARD);
    }
    /**
     * Sets the speed of the entry motor
     * @param speed -1 to 1
     */
    public void setEntrySpeed(double speed) {
        entry.set(speed);
    }

    /**
     * Sets the speed of the curve motor
     * @param speed -1 to 1
     */
    public void setCurveSpeed(double speed) {
        curve.set(speed);
    }

    /**
     * Sets the speed of the lower vertical motor
     * @param speed -1 to 1
     */
    public void setVerticalLowerSpeed(double speed) {
        verticalLower.set(speed);
    }

    /**
     * Sets the speed of the upper vertical motor
     * @param speed -1 to 1
     */
    public void setVerticalUpperSpeed(double speed) {
        verticalUpper.set(MathUtil.clamp(speed, -1, 1));
    }

    /**
     * Sets the speeds of all of the motors
     * @param entrySpeed -1 to 1 - Entry motor
     * @param curveSpeed -1 to 1 - Curve motor
     * @param verticalLowerSpeed -1 to 1 - Vertical lower motor
     * @param verticalUpperSpeed -1 to 1 - Vertical upper motor
     */
    public void setSpeeds(double entrySpeed, double curveSpeed, double verticalLowerSpeed, double verticalUpperSpeed) {
        entry.set(MathUtil.clamp(entrySpeed, -1, 1));
        curve.set(MathUtil.clamp(curveSpeed, -1, 1));
        verticalLower.set(MathUtil.clamp(verticalLowerSpeed, -1, 1));
        verticalUpper.set(MathUtil.clamp(verticalUpperSpeed, -1, 1));
    }

    /**
     * @return true if sensor 1 is activated (entry sensor of the entry intake)
     */
    public boolean isSensorOneActivated() {
        return sensor1.get();
    }

    /**
     * @return true if sensor 2 is activated (curve point)
     */
    public boolean isSensorTwoActivated() {
        return sensor2.get();
    }

    /**
     * @return true if sensor 3 is activated (lower of the vertical conveyor)
     */
    public boolean isSensorThreeActivated() {
        return sensor3.get();
    }

    /**
     * @return true if sensor 4 is activated (middle of the vertical conveyor)
     */
    public boolean isSensorFourActivated() {
        return sensor4.get();
    }

    /**
     * @return true if sensor 5 is activated (upper top of vertical conveyor)
     */
    public boolean isSensorFiveActivated() {
        return sensor5.get();
    }

    /**
     * Returns the amount of balls in the magazine, evaluated by the sensors.
     * @return the amount of sensors that are detecting something (0-5)
     */
    public int getAmountOfBalls() {
        boolean[] sensors = {isSensorOneActivated(), isSensorTwoActivated(), isSensorThreeActivated(), isSensorFiveActivated(), isSensorFourActivated()};
        int count = 0;
        for(boolean b : sensors) {
            if(b) count++;
        }
        return count;
    }

    /**
     * Returns a boolean array of all of the sensors
     * Index range is 0-4 (size:5)
     * @return boolean array
     */
    public boolean[] getSensors() {
        boolean[] sensors = {isSensorOneActivated(), isSensorTwoActivated(), isSensorThreeActivated(), isSensorFourActivated(), isSensorFiveActivated()};
        return sensors;
    }

    /**
     * Returns true if the intake is open
     * @return true if intake is open
     */
    public boolean isIntakeOpen() {
        return piston.get() == Value.kForward;
    }

    /**
     * Sets the intake position to either open or closed
     * @param toOpen - toOpen = true if you want piston to go forward and open the intake entry
     *                        = false if you want piston to retract and close the intake entry  
     */
    public void setIntakePiston(boolean toOpen) {
        piston.set(toOpen ? Value.kForward : Value.kReverse);
    }

}