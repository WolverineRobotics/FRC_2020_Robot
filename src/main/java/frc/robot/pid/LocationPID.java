package frc.robot.pid;

import edu.wpi.first.wpilibj.controller.PIDController;

public class LocationPID extends PIDController {

    private LocationCalculatorCurved locationCalc;
    
    public LocationPID(double kP){
        this(kP, 0);
    }

    public LocationPID(double kP, double kI){
        this(kP, kI, 0);
    }

    public LocationPID(double kP, double kI, double kD){
        super(kP, kI, kD);
    }

    // Must be called before starting
    public void initialize(double startingDistance, double startingHeading){
        this.locationCalc = new LocationCalculatorCurved(startingDistance, startingHeading, 0.75);
        enableContinuousInput(0, 360);
        reset();
    }

    public double getDistance(){
        double[] distanceDirection = locationCalc.getDistanceDirection();
        return distanceDirection[0];
    }

    // Use getDistance to determine distance
    public double calculate(double currentGyroAngle, double distanceChange) {
        locationCalc.update(distanceChange, currentGyroAngle);
        double[] distanceDirection = locationCalc.getDistanceDirection();
        setSetpoint(distanceDirection[1]);
        return super.calculate(currentGyroAngle);
    }
}