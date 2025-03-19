package frc.robot.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.RobotContainer;


public class InterpolatedPS4Gamepad extends PS4Controller {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    public InterpolatedPS4Gamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -deadZoneThreshold) && (axis < deadZoneThreshold);
    }

    public static boolean isCeiling(double axis) {
        return axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold;
    }

    public double interpolatedLeftYAxis() {
        if (Math.abs(this.getLeftY()) <= 0.03)
            return 0.0;
        if (RobotContainer.elevator.getElevatorPosition() > 7 ) {

            return ((Math.sin(this.getLeftY())) * .3);


        }
        
        return ((Math.sin(this.getLeftY())) * 1.2);
    }

    public double interpolatedLeftXAxis() {
        if (Math.abs(this.getLeftX()) <= 0.03)
            return 0.0;

          
           


            if (RobotContainer.elevator.getElevatorPosition() > 9 ) {

                return ((Math.sin(this.getLeftX())) * .40);
    
    
            }


            if (RobotContainer.elevator.getElevatorPosition() > 10 ) {

                return ((Math.sin(this.getLeftX())) * .35);
    
    
            }

            if (RobotContainer.elevator.getElevatorPosition() > 15 ) {

                return ((Math.sin(this.getLeftX())) * .30);
    
    
            }



            if (RobotContainer.elevator.getElevatorPosition() > 20 ) {

                return ((Math.sin(this.getLeftX())) * .25);
    
    
            }


            if (RobotContainer.elevator.getElevatorPosition() > 25 ) {

                return ((Math.sin(this.getLeftX())) * .20);
    
    
            }
            if (RobotContainer.elevator.getElevatorPosition() > 30 ) {

                return ((Math.sin(this.getLeftX())) * .15);
    
    
            }

            if (RobotContainer.elevator.getElevatorPosition() > 35 ) {

                return ((Math.sin(this.getLeftX())) * .10);
    
    
            }

            if (RobotContainer.elevator.getElevatorPosition() > 40 ) {

                return ((Math.sin(this.getLeftX())) * .05);
    
    
            }

            
        return ((Math.sin(this.getLeftX())) * .6);
    }

    // ps4 user 
    public double interpolatedRightXAxis() {
        if (Math.abs(this.getRightX()) <= 0.03)
            return 0.0;
        return (Math.sin(this.getRightX()) * -1.0
        );
    }

    
    




    // public double getLeftX() {
    //     if (Math.abs(this.getLeftX()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getLeftX();
    // }

    // public double getRightX() {
    //     if (Math.abs(this.getRightX()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getRightX();
    // }

    // public double getLeftY() {
    //     if (Math.abs(this.getLeftY()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getLeftY();
    // }

    public double interpolatedRightYAxis() {
        if (Math.abs(this.getRightY()) <= 0.05) {
            return 0.0;
        }
        return super.getRightY();
    }

}