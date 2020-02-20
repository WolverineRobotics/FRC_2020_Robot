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
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.RobotConst;
import frc.robot.constants.RobotMap;
import frc.robot.util.Util;

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
            if(isSensorOneActivated()) {
                boolean isNewBall = true;
                for(Ball b : mag) {
                    if(b.getCurrentPosition() == Position.ONE) {
                        isNewBall = false;
                    }
                }
                if(isNewBall) {
                    Ball ball = new Ball(getNextEmptyPosition());
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
     * Returns the amount of balls in the magazine, evaluted by the size of the magazine List<>
     * @return the amount of balls in the magazine
     */
    public int getAmountOfBalls() {
        int count = mag.size();
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
        Position lowestEmptyPosition = poses.get(poses.size() - 1);
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

    /**
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

    private List<Ball> unfinishedDesto = new ArrayList<Ball>(); //contains a list of balls who's destination is not actually the final destination, but take detours to get to their final destination.
    private List<Ball> ballsToRemove = new ArrayList<Ball>();
    private void executeMotors() {

        List<Ball> localMag = new ArrayList<>(this.mag);
        Collections.reverse(localMag);
        //(1) loop through each ball from top to bottom
        for(Ball ball : localMag) {
            //(2) evaluate next ball if ball is already at destination
            if(ball.isAtDestination()) {
                continue;
            }

            Position currentPos = ball.getCurrentPosition(); //TODO change occurences of this
            //(4) checks if this is third ball
            if(localMag.size() == 3) { //if there are 3 balls in magazine right now.
                if(ball == localMag.get(1)) { //if this current ball (in for loop) is the second ball in the mag (aka the ball at Position 4)
                    Ball ball2 = ball;
                    Ball ball3 = localMag.get(0);
                    if(unfinishedDesto.contains(ball2) || unfinishedDesto.contains(ball3)) {
                        if(ball2.getCurrentPosition() == Position.FOUR && ball3.getCurrentPosition() == Position.ONE) {
                            ball2.setDestination(Position.TWO);
                            ball3.setDestination(Position.ONE);
                        } else if(ball2.getCurrentPosition() == Position.TWO && ball3.getCurrentPosition() == Position.ONE) {
                            ball2.setDestination(Position.FOUR);
                            ball3.setDestination(Position.THREE);
                        } else if(ball2.getCurrentPosition() == Position.FOUR && ball3.getCurrentPosition() == Position.THREE) {
                            unfinishedDesto.remove(ball2);
                            unfinishedDesto.remove(ball3);
                        }
                    }
                }
            }

            //evaluate the desired motor power based on whether the destination is after or before the current position.
            Position destination = ball.getDestination();
            int direction = 1;
            if(Position.isAfter(currentPos, destination)) {
                direction = -1;
            }

            Motor[] possessions = currentPos.getPossessions();
            for(Motor possession : possessions) {
                switch (possession) {
                    case ENTRY:
                        setEntrySpeed(direction*RobotConst.IntakeConst.ENTRY_SPEED);
                        break;
                    case CURVE:
                        setCurveSpeed(direction*RobotConst.IntakeConst.CURVE_SPEED);
                        break;
                    case LOWER_VERTICAL:
                        setVerticalLowerSpeed(direction*RobotConst.IntakeConst.LOWER_VERTICAL_SPEED);
                        break;
                    case UPPER_VERTICAL:
                        setVerticalUpperSpeed(direction*RobotConst.IntakeConst.UPPER_VERTICAL_SPEED);
                        break;
                }
            }

            //re-evaluate current ball position based on sensor feedback
            double posId = currentPos.getId();
            double destinationId = destination.getId();
            boolean[] sen = getSensors();

            if(direction == 1) { //motor positive
                //checking if the current location of the ball has a sensor
                if(Util.isInteger(currentPos.getId())) {
                    int posIdInt = (int) posId;
                    boolean isDetectingBall = sen[posIdInt - 1]; //gets the sensor of that pos
                    if(!isDetectingBall) {
                        Position newPosition = Position.getPosition(posIdInt+0.5);
                        if(newPosition == Position.FIVE_SIX) {
                            ballsToRemove.add(ball);
                        }
                        ball.setPosition(newPosition);
                    }
                }
            } else { //motor negative
                if(Util.isInteger(currentPos.getId())) {
                    int posIdInt = (int) posId;
                    boolean isDetectingBall = sen[posIdInt - 1]; //gets the sensor of that pos
                    if(!isDetectingBall) {
                        Position newPosition = Position.getPosition(posIdInt-0.5);
                        if(newPosition == Position.FIVE_SIX) {
                            ballsToRemove.add(ball);
                        }
                        ball.setPosition(newPosition);
                    }
                }
            }


        }
        Collections.reverse(localMag);
        this.mag = localMag;
    }

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

        public boolean isAtDestination() {
            return position == destination;
        }

        public void setDestination(Position destination) {
            this.destination = destination;
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
        public static Position getIntLocationAfter(Position pos) {
            double id = pos.getId();
            if(id > 5) {
                return null;
            }
            double desiredId = Math.ceil(id + 1);
            return map.get(desiredId);
        }

        /**
         * 
         * @param pos pos to check
         * @param checkPos checking if this pos is after pos
         * @return true if checkPos is enum after pos.
         */
        public static boolean isAfter(Position pos, Position checkPos) {
            double posId = pos.getId();
            double checkPosId = checkPos.getId();
            return checkPosId > posId;
        }

        /**
         * Get the Position enum based on id
         * @return the position with that corresponding id, or can return null
         */
        public static Position getPosition(double id) {
            for(Map.Entry<Double, Position> entry : map.entrySet()) {
                double entryId = entry.getKey();
                if(entryId == id) {
                    return entry.getValue();
                }
            }
            return null;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("[BALL SENSOR 1]", this::isSensorOneActivated, null);
    }

}