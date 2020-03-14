package frc.robot.pid;

import frc.robot.util.Util;
import frc.robot.constants.RobotConst.PIDConst;

class LocationCalculatorCurved extends LocationCalculator {

    protected double startingDistance;
    protected double startingAngle;
    protected double[] startingLocation;

    // Value between 0 and 1
    // The greater it is, the more aggressively the robot will turn and curve
    private double setPointDistance;
    private final static double DEFAULT_SETPOINT_DISTANCE = PIDConst.LOCATIONCALC_DEFAULT_SETPOINT_DISTANCE;

    public LocationCalculatorCurved(double distance, double angle, double setPointDistance) {
        super(distance, angle);
        this.startingDistance = distance;
        this.startingAngle = angle;
        // this.startingLocation = polarToCartesian(distance, angle);
        this.startingLocation = this.location;

        // Ensures that the setPointDistance is between 0 and 1
        setPointDistance = Util.zeroToOne(setPointDistance);

        this.setPointDistance = setPointDistance;
    }

    public LocationCalculatorCurved(double distance, double angle) {
        this(distance, angle, DEFAULT_SETPOINT_DISTANCE);
    }

    @Override
    public double[] getDistanceDirection() {

        double currentDistance = pythagorean(location[0], location[1]);

        // Calculates how much of the original distance to the destination has been
        // traveled
        // Value between 0 and 1
        double amountTraveled = currentDistance / startingDistance;
        amountTraveled = Util.zeroToOne(amountTraveled);

        // A point on the original vector from the starting position to the destination
        // The distance from this point to the destination is the same as the current
        // distance from the destination.
        double[] correspondingOriginalPoint = { 0, 0 };

        for (int i = 0; i < 2; i++) {
            correspondingOriginalPoint[i] = startingLocation[i] * amountTraveled;
        }

        // A point on the original vector from the starting position to the destination
        // It is the correspoindingOriginalPoint multiplied by a scaling factor,
        // setPointDistance
        double[] tempSetpoint = { 0, 0 };
        for (int i = 0; i < 2; i++) {
            tempSetpoint[i] = correspondingOriginalPoint[i] * setPointDistance;
        }

        double[] destoPolar = cartesianToPolar(tempSetpoint);
        destoPolar[1] = angleWithinBounds(destoPolar[1] + 180);

        return destoPolar;

    }

}