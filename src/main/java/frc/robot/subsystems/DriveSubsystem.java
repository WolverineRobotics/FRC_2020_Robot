package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private Spark leftDrive01, leftDrive02, rightDrive01, rightDrive02;
    private Encoder leftEncoder, rightEncoder;



    public DriveSubsystem(){
        leftDrive01 = new Spark(DRIVE_LEFT_MOTOR_MASTER_ADDRESS);
        leftDrive02 = new Spark(DRIVE_LEFT_MOTOR_SLAVE_ADDRESS);
        rightDrive01 = new Spark(DRIVE_RIGHT_MOTOR_MASTER_ADDRESS);
        rightDrive02 = new Spark(DRIVE_RIGHT_MOTOR_SLAVE_ADDRESS);

        leftEncoder = new Encoder(DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B);
        rightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B);
    }
    
    public void setLeftSpeed(double speed){
        leftDrive01.set(speed);
        leftDrive02.set(speed);
    }

    public void setRightSpeed(double speed){
        rightDrive01.set(speed);
        rightDrive02.set(speed);
    }

    public void setSpeed(double speed){
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    /**
     * returns the absolute raw encoder counts of the left encoder
     * @return int number of counts to deliver
     */
    public int getRawLeftEncoder() {
        return leftEncoder.get();
    }

    /**
     * returns the absolute encoder counts of the right encoder
     * @return int number of counts to deliver
     */
    public int getRawRightEncoder() {
        return rightEncoder.get();
    }

    public double getLeftSpeed() {
        return (leftDrive01.get() + leftDrive02.get()) /2;
    }

    public double getRightSpeed() {
        return (rightDrive01.get() + rightDrive02.get()) / 2;
    }


}