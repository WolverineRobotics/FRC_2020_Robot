package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConst;
import frc.robot.constants.RobotMap;
import frc.robot.util.Util;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * Ball sensors detect whether a certain ball is in position
     */
    private DigitalInput sensor1; //entry sensor of the intake (ball5 location)
    private DigitalInput sensor2; //sensor located at curve (ball4 location)
    private DigitalInput sensor3; //sensor located at bottom of vertical conveyor (ball3 location)
    private DigitalInput sensor4; //sensor located at middle of vertical conveyor (ball2 location)
    private DigitalInput sensor5; //sensor located at top of vertical conveyor (ball1 location)

    /**
     * Motors control certain conveyor systems. 
     */
    private CANSparkMax entry; // main intake, entry point of ball
    private CANSparkMax curve; // curves ball into vertical conveyor
    private CANSparkMax verticalLower; // controls vertical conveyor lower
    private CANSparkMax verticalUpper; //controls vertical conveyor upper 

    private DoubleSolenoid piston; //piston that opens the front intake. Forward=Intake Openened and Reverse=Intake Closed

    /**
     * Variables helping with evaluating ball states.
     */
    public List<Ball> mag;
    public List<Ball> unfinishedDesto; //contains a list of balls who's destination is not actually the final destination, but take detours to get to their final destination.
    public List<Ball> ballsToRemove; //contains a list of balls to remove after the executeMotors() method is done. Balls put in here are balls that were shot out by the Shooter Subsystem
    public List<Motor> currentPossessions;
    public boolean moveBalls;

    // initializes all of the components within the subsystem
    public IntakeSubsystem() {
        super();
        sensor1 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_1);
        sensor2 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_2);
        sensor3 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_3);
        sensor4 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_4);
        sensor5 = new DigitalInput(RobotMap.Sensors.BALL_SENSOR_5);

        entry = new CANSparkMax(RobotMap.SpeedController.ENTRY, MotorType.kBrushless);
        curve = new CANSparkMax(RobotMap.SpeedController.CURVE, MotorType.kBrushless);

        curve.setInverted(true);

        verticalLower = new CANSparkMax(RobotMap.SpeedController.VERTICAL_LOWER, MotorType.kBrushless);
        verticalUpper = new CANSparkMax(RobotMap.SpeedController.VERTICAL_UPPER, MotorType.kBrushless);

        piston = new DoubleSolenoid(RobotMap.Pneumatic.INTAKE_FORWARD, RobotMap.Pneumatic.INTAKE_BACKWARD);

        mag = new ArrayList<>();
        unfinishedDesto = new ArrayList<Ball>();
        ballsToRemove = new ArrayList<Ball>();
        currentPossessions = new ArrayList<>();
        moveBalls = false;
    }

    /**
     * Runs periodically while the Robot is enabled.
     */
    @Override
    public void periodic() {
        if(moveBalls) {
            if(mag.size() != 5) setSpeeds(0.3, 0, 0, 0);
            if(isSensorOneActivated()) {
                boolean isNewBall = true;
                for(Ball b : mag) {
                    if(b.getCurrentPosition() == Position.ONE) {
                        isNewBall = false;
                        break;
                    }
                }

                if(isNewBall && !RobotContainer.getOperatorController().isOutaking()) {
                    Ball ball = new Ball(getNextEmptyPosition());
                    mag.add(ball);
                    if(mag.size() == 2) {
                        ball.setDestination(Position.THREE);
                        unfinishedDesto.add(ball);
                    } else if(mag.size() == 3) {
                        ball.setDestination(Position.TWO);
                        unfinishedDesto.add(ball);
                    } else if(mag.size() == 4) {
                        ball.setDestination(Position.TWO);
                    } else if(mag.size() == 5) {
                        ball.setDestination(Position.ONE);
                    }
                }
            }
            executeMotors();
            stopUnusedPossessions();
        }
        updateSensorPositions();
        updateDashboard();
    }

    private void updateDashboard() {
        //display all sensor values
        boolean[] sen = getSensors();
        for(int i = 0; i < sen.length; i++) {
            SmartDashboard.putBoolean("Sensor " + (i+1), sen[i]);
        }

        //display all of the ball object data
        for(int i = 0; i < mag.size(); i++) {
            SmartDashboard.putString("Ball #" + (i+1) + " Destination:", mag.get(i).getDestination().toString());
            SmartDashboard.putString("Ball #" + (i+1) + " Position:", mag.get(i).getCurrentPosition().toString());
        }

        // SmartDashboard.putString("Next Available Position", getNextEmptyPosition().toString());
        SmartDashboard.putNumber("Magazine Amount", mag.size());
        
    }


    /**
     * Rough (not exactly how the code was written but roughly)
     * Algorithm:
     * 1. For every ball in the magazine (in top to bottom order - must loop in this fashion)
     * 2. If the ball is already at destination, go onto next ball
     * 3. Check if the ball's destination is clear **unclear of what this means
     * 4. Check if this ball is ball #3
     *    > If so, change ball #2's destination to Position.TWO
     *    > Set ball #3's destination to Position.ONE
     *    > Once they are both in that position, set ball #2's position to Position.FOUR and #3's to Position.THREE
     * 5. Bring the ball to next empty position. Check if the ball's destination is negative motor power or positive motor power.
     * 6. Check if the ball's current position is an integer
     *  > if it an integer, then check if that corresponding sensor has something.
     *      > if that corresponding sensor does not have something and the next one doesn't have something as well as no other ball has the next one's position, check if the motor speed was positive or negative
     *          > set that current motor possession speed to zero
     *          > if it was positive then the update position is +0.5, otherwise if it is negative -0.5
     *        
     */

    private void executeMotors() {
        for(Ball b : mag) {
            if(!b.isAtDestination()) {
                Position currentPos = b.getCurrentPosition();
                Position desto = b.getDestination();
                int direction = 1;
                if(Position.isAfter(currentPos, desto)) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                // System.out.println("Direction: " + direction);
                // run all of the motor possessions
                Motor[] possessions = currentPos.getPossessions();
                for(Motor possession : possessions) {
                    if(!currentPossessions.contains(possession)) {
                        switch (possession) {
                            case ENTRY:
                                setEntrySpeed(direction*0.3);
                                break;
                            case CURVE:
                                setCurveSpeed(direction*0.5);
                                break;
                            case LOWER_VERTICAL:
                                boolean[] sen = getSensors();
                                if((sen[3-1] && sen[4-1]) && mag.size() == 3) {
                                    if(isPositionOccupied(Position.THREE) && isPositionOccupied(Position.TWO)) {
                                        Ball ball2 = getBall(Position.THREE);
                                        Ball ball3 = getBall(Position.TWO);
                                        if(ball2 != null && ball3 != null) {
                                            ball2.setPosition(Position.FOUR);
                                            ball3.setPosition(Position.THREE);
                                        }
                                    }
                                } else {
                                    setVerticalLowerSpeed(direction*0.15);
                                }
                                break;
                            case UPPER_VERTICAL:
                                setVerticalUpperSpeed(direction*0.15);
                                break;
                        }
                    }
                }
                
            }
        }
    }

    /**
     * Stops unused motor possessions
     */
    private void stopUnusedPossessions() {
        // for(Ball ball : mag) {
        //     if(ball.isAtDestination()) {
        //         for(Motor m : ball.getCurrentPosition().getPossessions()) {
        //             // if(ball.getCurrentPosition().getPossessions()[0] == ) {
        //                 switch(m) {
        //                     case ENTRY:
        //                         setEntrySpeed(0);
        //                         break;
        //                     case CURVE:
        //                         setCurveSpeed(0);
        //                         break;
        //                     case LOWER_VERTICAL:
        //                         // setVerticalLowerSpeed(0);
        //                         break;
        //                     case UPPER_VERTICAL:
        //                         setVerticalUpperSpeed(0);
        //                         break;
        //                 }
        //                 currentPossessions.remove(m);
        //             // }
        //         }
        //     }
        // }
        for(Ball ball : mag) {
            if(ball.isAtDestination() /*&& !unfinishedDesto.contains(ball)*/) {
                for(Motor m : ball.getCurrentPosition().getPossessions()) {
                    switch(m) {
                        case ENTRY:
                            setEntrySpeed(0);
                            break;
                        case CURVE:
                            if(mag.size() >= 3 && ball.getCurrentPosition() == Position.THREE) {

                            } else {
                                setCurveSpeed(0);   
                            }
                            break;
                        case LOWER_VERTICAL:
                            setVerticalLowerSpeed(0);
                            break;
                        case UPPER_VERTICAL:
                            setVerticalUpperSpeed(0);
                            break;
                    }
                }
            }
        }
    }

    /**
     * Gets ball object with corresponding Position
     * @param pos Position enum
     * @return Ball object. Can return null if no ball is found at that position
     */
    public Ball getBall(Position pos) {
        for(Ball b : mag) {
            if(b.getCurrentPosition() == pos) {
                return b;
            }
        }
        return null;
    }

    /**
     * Updates Ball Position based on sensor logic
     */
    private void updateSensorPositions() {
        for(Ball b : mag) {
            if(!b.isAtDestination()) {
                Position position = b.getCurrentPosition();
                double positionId = position.getId();
                boolean[] sen = getSensors();
                if(Util.isInteger(positionId)) {
                    int intPosId = (int) positionId;
                    if(sen[intPosId - 1]) {

                    } else {
                        List<Position> occupied = getOccupiedPositions();
                        boolean nextSensor = sen[intPosId];
                        boolean nextPosNotOccupied = !occupied.contains(getPosition(intPosId + 1));
                        if(nextSensor && (nextPosNotOccupied)) {
                            b.setPosition(getPosition(intPosId + 1));
                            // System.out.println("============================================================");
                        }
                        if(mag.size() == 3 && mag.get(1) == b) {
                            Ball ball2 = b;
                            Ball ball3 = mag.get(2);
                            if(ball2.getCurrentPosition() == Position.FOUR) {
                                ball3.setPosition(Position.THREE);
                            }
                        }
                    }
                }
            } else if(unfinishedDesto.contains(b)) {
                if(mag.size() == 3) {
                    if(b.getCurrentPosition() == Position.THREE) {
                       Ball ball3 = mag.get(2);
                       if(ball3.getCurrentPosition() == Position.TWO) {
                           b.setDestination(Position.FOUR);
                           ball3.setDestination(Position.THREE);
                           unfinishedDesto.remove(b);
                           unfinishedDesto.remove(ball3);
                       }
                    
                    // if(b.getCurrentPosition() == Position.FOUR && mag.size() == 3 && mag.get(1) == b) {
                    //     Ball ball3 = mag.get(2);
                    //     ball3.setPosition(Position.THREE);
                    //     ball3.setDestination(Position.THREE);
                    // }
                    }
                }
            } else {
                Position currentPos = b.getCurrentPosition();
                Position destination = b.getDestination();
                // if(Position.isAfter(currentPos, destination) && currentPos) {

                // }

            }
        }
    }

    /**
     * Check whether you want balls to re-evaluate themselves
     * @param toMove true - if you want them to move, false otherwise.
     */
    public void setMoveBalls(boolean toMove) {
        this.moveBalls = toMove;
    }
    
    /**
     * Sets the speed of the entry motor
     * @param speed -1 to 1
     */
    public void setEntrySpeed(double speed) {
        entry.set(speed);
    }

    /**
     * Gets the speed of the entry motor
     * @return -1 to 1
     */
    public double getEntrySpeed() {
        return entry.get();
    }

    /**
     * Sets the speed of the curve motor
     * @param speed -1 to 1
     */
    public void setCurveSpeed(double speed) {
        curve.set(speed);
    }

    /**
     * Returns the curve speed conveyor
     * @return speed double -1 to 1
     */
    public double getCurveSpeed() {
        return curve.get();
    }

    /**
     * Sets the speed of the lower vertical motor
     * @param speed -1 to 1
     */
    public void setVerticalLowerSpeed(double speed) {
        verticalLower.set(speed);
    }

    /**
     * Returns Lower Vertical Conveyor Motor Speed
     * @return speed double (-1 to 1)
     */
    public double getVerticalLowerSpeed() {
        return verticalLower.get();
    }

    /**
     * Sets the speed of the upper vertical motor
     * @param speed -1 to 1
     */
    public void setVerticalUpperSpeed(double speed) {
        verticalUpper.set(MathUtil.clamp(speed, -1, 1));
    }

    /**
     * Gets the vertical upper conveyor speed
     * @return upper vertical conveyor speed
     */
    public double getVerticalUpperSpeed() {
        return verticalUpper.get();
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
        if(isIntakeOpen()) {
            return false;
        } else {
            return !sensor1.get();
        }
    }

    /**
     * @return true if sensor 2 is activated (curve point)
     */
    public boolean isSensorTwoActivated() {
        return !sensor2.get();
    }

    /**
     * @return true if sensor 3 is activated (lower of the vertical conveyor)
     */
    public boolean isSensorThreeActivated() {
        return !sensor3.get();
    }

    /**
     * @return true if sensor 4 is activated (middle of the vertical conveyor)
     */
    public boolean isSensorFourActivated() {
        return !sensor4.get();
    }

    /**
     * @return true if sensor 5 is activated (upper top of vertical conveyor)
     */
    public boolean isSensorFiveActivated() {
        return !sensor5.get();
    }

    /**
     * Returns the amount of balls in the magazine, evaluted by the size of the magazine List<>
     * @return the amount of balls in the magazine
     */
    public int getAmountOfBalls() {
        int count = mag.size();
        return count;
    }

    /**
     * Returns if the ball magazine is empty
     * @return true if ball magazine is empty
     */
    public boolean isMagazineEmpty() {
        return mag.isEmpty();
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
     * 
     * If 2 and 5 are occupied, return 1.
     */
    public Position getNextEmptyPosition() {
        List<Position> occupied = getOccupiedPositions();
        if(occupied.isEmpty()) {
            return Position.FIVE;
        } else {
            Position currentLowest = Position.FIVE;
            for(Position pos : occupied) {
                if(!Position.isAfter(pos, currentLowest)) {
                    currentLowest = pos;
                }
            }
            return currentLowest;
        }
    }

    /**
     * Check whether a ball is taking up a position
     * @param pos Position enum
     */
    public boolean isPositionOccupied(Position pos) {
        for(Ball ball : mag) {
            if(ball.getCurrentPosition() == pos) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns a List<> of occupied positions
     * @return List<Position> of occupied positions by balls.
     */
    private List<Position> getOccupiedPositions() {
        List<Position> pos = new ArrayList<Position>();
        for(Ball b : mag) {
            pos.add(b.getCurrentPosition());
        }
        return pos;
    }

    /**
     * Ball class
     * has two main variables: destination & position.
     * Simply has its own getter & seter for each
     * Also automatically sets its position to Position.ONE
     * because instantiating a new ball has to come from the front of the intake.
     */
    public class Ball {

        private Position destination;
        private Position position;

        public Ball(Position destination) {
            this.destination = destination;
            this.position = Position.ONE;
        }

        public Position getCurrentPosition() {
            return position;
        }

        public void setPosition(Position pos) {
            position = pos;
        }

        public Position getDestination() {
            return destination;
        }

        public void setDestination(Position destination) {
            this.destination = destination;
        }

        public boolean isAtDestination() {
            return position == destination;
        }

    }

    /**
     * Position enum
     * Has ids and motor possessios.
     * Each Ball Position has its own motor possession.
     * 
     * Also has a static component which keeps track of all other enums
     * so you can back track, forward track, get the position enum based off id,
     * and check if a position is after another position.
     */
    public enum Position {
        ONE(1.0, Motor.ENTRY, Motor.CURVE),
        ONE_TWO(1.5, Motor.CURVE),
        TWO(2.0, Motor.CURVE, Motor.LOWER_VERTICAL),
        TWO_THREE(2.5, Motor.LOWER_VERTICAL),
        THREE(3.0, Motor.CURVE, Motor.LOWER_VERTICAL),
        THREE_FOUR(3.5, Motor.LOWER_VERTICAL),
        FOUR(4.0, Motor.LOWER_VERTICAL, Motor.UPPER_VERTICAL),
        FOUR_FIVE(4.5, Motor.LOWER_VERTICAL),
        FIVE(5.0, Motor.UPPER_VERTICAL),
        FIVE_SIX(5.5, Motor.UPPER_VERTICAL),
        ;

        private double id;
        private Motor[] possessions;

        Position(double id, Motor... possession) {
            this.id = id;
            possessions = possession;
        }

        /**
         * Get all of the motor possessions that this Position has.
         * 
         * @return array of Motor
         */
        public Motor[] getPossessions() {
            return possessions;
        }

        /**
         * Returns the id that is defined in the enum declaration
         */
        public double getId() {
            return id;
        }

        /**
         * Static componnet: Keeps a map of all members of this enum.
         */
        private static Map<Double, Position> map = new HashMap<>();

        static {
            for (Position pos : Position.values()) {
                map.put(pos.getId(), pos);
            }
        }

        /**
         * 
         * Get the integer position before the position given.
         * 
         * @param pos Position of the ball
         * @return the Position of the integer location before
         * 
         *         Example: Argued Position: 1.5 Returns: 1.0
         * 
         *         Another example: Argued Position: 4.0 Returns: 3.0
         */
        public static Position getIntLocationBefore(Position pos) {
            double id = pos.getId();
            double desiredId = Math.ceil(id - 1);
            return map.get(desiredId);
        }

        /**
         * 
         * Get the integer position before the position given.
         * 
         * @param pos Position of the ball. CANNOT BE GREATER THAN 5
         * @return the Position of the integer location before
         * 
         *         Example: Argued Position: 1.5 Returns: 2
         * 
         *         Another example: Argued Position: 4.0 Returns: 5
         */
        public static Position getIntLocationAfter(Position pos) {
            double id = pos.getId();
            if (id > 5) {
                return null;
            }
            double desiredId = Math.ceil(id + 1);
            return map.get(desiredId);
        }

        /**
         * Checks if a Position is after another Position.
         * Use instructions: Position.isAfter().
         * Note that this is a static method.
         * @param pos      pos to check
         * @param checkPos checking if this pos is after pos
         * @return true if checkPos is enum after pos.
         */
        public static boolean isAfter(Position pos, Position checkPos) {
            double posId = pos.getId();
            double checkPosId = checkPos.getId();
            return checkPosId > posId;
        }
    }

    /**
     * See #Position enum Use of this enum is just so that there is a better sense
     * of logic in the code.
     */
    public enum Motor {
        ENTRY, CURVE, LOWER_VERTICAL, UPPER_VERTICAL,;
    }

    /**
     * Gets the position by ID (1 - 5.5)
     * @param id (double) position ID
     * @return the position. Can return null if position with id is not found.
     */
    private Position getPosition(double id) {
        for(Position pos : Position.values()) {
            if(pos.getId() == id) {
                return pos;
            }
        }
        return null;
    }

}