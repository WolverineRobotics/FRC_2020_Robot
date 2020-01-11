package frc.robot.pid;


class LocationCalculator{


    // Array containing [x,y] values for where the robot is in relation to the destination.
    // Destination is (0, 0)
    // Negative x-axis means that the robot must go towards heading 0 to reach destination.
    // Positive x-axis means that from the destination, the robot is at heading direction 0 and
    // must travel at heading 180 to reach destination.
    protected double[] location;


    
    public LocationCalculator(double distance, double angle){
        this.location = polarToCartesian(distance, angleWithinBounds(angle + 180));
        // angle + 180 to get the angle in the opposite direction.
        // The arguments are for where the destination is in relation to the robot.
        // this.location is for where the robot is in relation to the destination
    }

    public void update(double distanceChange, double angle){

        double[] change = this.polarToCartesian(distanceChange, angle);
        for(int i=0;i<2;i++){
            this.location[i] += change[i];
        }
    }

    // Returns the distance and direction in an array. Index 0 is distance, index 1 is gyro heading.
    public double[] getDistanceDirection(){


        double[] robotLocation = this.cartesianToPolar(this.location);
        double[] destinationLocation = {robotLocation[0], angleWithinBounds(robotLocation[1] + 180 )};

        // Print statements for debugging
        System.out.println("X VALUE = " + this.location[0]);
        System.out.println("Y VALUE = " + this.location[1]);
        System.out.println("DISTANCE = " + destinationLocation[0]);
        System.out.println("ANGLE = " + destinationLocation[1]);

        return destinationLocation;
    }



    // Pythagorean theorem, takes the legs as args, returns hypotenuse 
    // Used to calculate cartesian distance without converting to polar
    protected double pythagorean(double num1, double num2){
        return Math.sqrt(Math.pow(num1,2) + Math.pow(num2, 2));
    }

     // Ensures the angle is between 0 and 360
    protected double angleWithinBounds(double angle){
        angle = angle % 360;
        // Since % of a (-) num in java gives a (-) num, this turns it positive
        if (angle < 0){
            angle += 360;
        }
        return angle;
    }

    protected double[] polarToCartesian(double distance, double angle){
        
        double x = Math.cos(Math.toRadians(angle))*distance;
        double y = Math.sin(Math.toRadians(angle))*distance;



        double[] cartesian = {x,y};
        return cartesian;
    }
    

    protected double[] cartesianToPolar(double[] cartesian){
        double x = cartesian[0];
        double y = cartesian[1];

        double distance = pythagorean(x, y); 
        double angle = Math.toDegrees(Math.atan(y/x));

        // Since tan(n) = tan(180 + n), there are 2 possibilities for the angle.
        // The angle given by arctan will be incorrect in quadrants 2 and 3, where x is negative.
        // This compensates for that.
        if (x < 0){
            angle += 180;
        }

        angle = angleWithinBounds(angle);

        double[] polar = {distance, angle};
        return polar;
    }


}