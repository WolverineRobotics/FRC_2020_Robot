package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst;
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

    private Position[] intPositions = {Position.ONE, Position.TWO, Position.THREE, Position.FOUR, Position.FIVE}; //contains integer positions (not the in-betweens)

    private List<Ball> mag;
    private boolean[] asteriks;
    private boolean moveBalls;

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

        mag = new ArrayList<>();
        asteriks = new boolean[5];
        moveBalls = false;
    }

    @Override
    public void periodic() {
        if(moveBalls) {
            boolean sen[] = {isSensorOneActivated(), isSensorTwoActivated(), isSensorThreeActivated(), isSensorFourActivated(), isSensorFiveActivated()};
            if(mag.isEmpty()) {
                if(sen[0]) { //must be first intaked ball
                    Ball ball = new Ball(Position.FIVE);
                    mag.add(ball);
                }
            }
            executeMotors();
        }
    }

    public void moveBalls() {
        moveBalls = true;
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
        entry.set(MathUtil.clamp(entrySpeed*RobotConst.IntakeConst.ENTRY_SPEED_REDUCTION_RATIO, -1, 1));
        curve.set(MathUtil.clamp(curveSpeed*RobotConst.IntakeConst.CURVE_SPEED_REDUCTION_RATIO, -1, 1));
        verticalLower.set(MathUtil.clamp(verticalLowerSpeed*RobotConst.IntakeConst.LOWER_VERTICAL_SPEED_REDUCTION_RATIO, -1, 1));
        verticalUpper.set(MathUtil.clamp(verticalUpperSpeed*RobotConst.IntakeConst.UPPER_VERTICAL_SPEED_REDUCTION_RATIO, -1, 1));
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

    public boolean isMagazineEmpty() {
        return mag.isEmpty();
    }

    /**
     * 
     * @return the next empty position on the intake.
     * For example:
     * 
     *      5 |
     *      4 |
     *      3 |
     *    1 2 |
     *    ____|
     * 
     *  Let's say that only number 5 has a ball.
     *  The next empty location will be 4.
     */
    public Position getNextEmptyPosition() {
        List<Position> poses = getOccupiedPositions();
        Position lowestEmptyPosition = poses.get(0);
        Position nextEmptyPosition = Position.getIntLocationBefore(lowestEmptyPosition);
        return nextEmptyPosition;
    }

    public boolean isPositionOccupied(Position pos) {
        for(Ball ball : mag) {
            if(ball.position == pos) {
                return true;
            }
        }
        return false;
    }

    private List<Position> getOccupiedPositions() {
        List<Position> pos = new ArrayList<Position>();
        mag.stream().forEach((x) -> pos.add(x.position));
        return pos;
    }

    private void executeMotors() {
        List<Ball> mag = new ArrayList<>(this.mag);
        Collections.reverse(mag);
        for(Ball ball : mag) {
            Position currentPos = ball.getCurrentPosition();
            Position nextEmptyPos = getNextEmptyPosition();
            if(nextEmptyPos != null) {
                for (Motor posession : currentPos.getPossessions()) {
                    switch(posession) {
                        case ENTRY:
                            setEntrySpeed(RobotConst.IntakeConst.ENTRY_SPEED);
                            break;
                        case CURVE:
                            setCurveSpeed(RobotConst.IntakeConst.CURVE_SPEED);
                            break;
                        case LOWER_VERTICAL:
                            setVerticalLowerSpeed(RobotConst.IntakeConst.LOWER_VERTICAL_SPEED);
                            break;
                        case UPPER_VERTICAL:
                            setVerticalUpperSpeed(RobotConst.IntakeConst.UPPER_VERTICAL_SPEED);
                            break;
                    }
                }
                boolean[] sen = getSensors();
                
            } else {

            }
        }
    }

    public class Ball {

        private Position destination;
        private Position position;

        public Ball(Position destination) {
            this.destination = destination;
            this.position = Position.ONE;
        }
     
        public void redefineDestination(Position destination) {
            this.destination = destination;
        }

        public Position getCurrentPosition() {
            return position;
        }

        public Position getDestination() {
            return destination;
        }

    }

    public enum Position {
        ONE(1.0, Motor.ENTRY),
        ONE_TWO(1.5, Motor.CURVE),
        TWO(2.0, Motor.CURVE),
        TWO_THREE(2.5, Motor.LOWER_VERTICAL),
        THREE(3.0, Motor.LOWER_VERTICAL),
        THREE_FOUR(3.5, Motor.LOWER_VERTICAL),
        FOUR(4.0, Motor.LOWER_VERTICAL),
        FOUR_FIVE(4.5, Motor.LOWER_VERTICAL),
        FIVE(5.0, Motor.UPPER_VERTICAL),
        FIVE_SIX(5.5, Motor.UPPER_VERTICAL),
        ;

        private double id;
        private Motor[] possessions;
        private static Map<Double, Position>  map = new HashMap<>();

        static {
            for(Position pos : Position.values()) {
                map.put(pos.getId(), pos);
            }
        }

        Position(double id, Motor... possession) {
            this.id = id;
            possessions = possession;
        }

        public Motor[] getPossessions() {
            return possessions;
        }

        /**
         * 
         * Get the integer position before the position given.
         * @param pos Position of the ball
         * @return the Position of the integer location before
         * 
         * Example:
         * Argued Position: 1.5
         * Returns: 1.0
         * 
         * Another example:
         * Argued Position: 4.0
         * Returns: 3.0
         */
        public static Position getIntLocationBefore(Position pos) {
            double id = pos.getId();
            double desiredId = Math.ceil(id - 1);
            return map.get(desiredId);
        }

        /**
         * 
         * Get the integer position before the position given.
         * @param pos Position of the ball. CANNOT BE GREATER THAN 5
         * @return the Position of the integer location before
         * 
         * Example:
         * Argued Position: 1.5
         * Returns: 2
         * 
         * Another example:
         * Argued Position: 4.0
         * Returns: 5
         */
        public static Position getIntLocationBefore(Position pos) {
            double id = pos.getId();
            if(id > 5) {
                return null;
            }
            double desiredId = Math.ceil(id + 1);
            return map.get(desiredId);
        }

        public double getId() {
            return id;
        }
    }  

    public enum Motor {
        ENTRY,
        CURVE,
        LOWER_VERTICAL,
        UPPER_VERTICAL,
        ;
    }



}