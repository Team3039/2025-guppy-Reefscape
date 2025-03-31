// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix.sensors.CANCoder;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;





public class Climb extends SubsystemBase {

// Declare a CANCoder instance

private final CANcoder wcpIsTheBest = new CANcoder(18);




  PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

 
  // Create the possible states of the climb
  public enum ClimbState {
    DISABLED,
    CLIMBING
  }

  // Create a variable to store the current state of the climb\]
  public ClimbState climbState = ClimbState.DISABLED;

  // Create a talonfx for the climb
  TalonFX climb = new TalonFX(TunerConstants.CLIMB);

  // Climb Constructor
  public Climb() {
  }

  /**
   * Get the state of the climb
   * 
   * @return the state of the climb as a ClimbState
   */
  public ClimbState getState() {
    return climbState;
  }

  /**
   * Set the state of the climb
   * 
   * @param state the state of the climb
   */
  public void setState(ClimbState state) {
    climbState = state;
  }

  public double getClimbPosition() {
    return wcpIsTheBest.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void periodic() {

    if (RobotContainer.climb.getClimbPosition() <= -0.17 && RobotContainer.climb.getClimbPosition() >= -0.16) {
      System.out.println("Hey I should stop");
  
      RobotContainer.climb.setState(ClimbState.DISABLED);
    }

    StatusSignal<Angle> encoderValue = wcpIsTheBest.getAbsolutePosition();


    SmartDashboard.putNumber("Climb Encoder Value", getClimbPosition());


    SmartDashboard.putString("climbState", String.valueOf(getState()));


    


    // Climb State Machine
    switch (climbState) {
      // In the disabled state, the climb is disabled
      case DISABLED:
        climb.disable();
        PDH.setSwitchableChannel(false);

        break;
      case CLIMBING:

      if (RobotContainer.climb.getClimbPosition() <= -0.17 && RobotContainer.climb.getClimbPosition() >= -0.16) {
        System.out.println("Hey I should stop");
    
        RobotContainer.climb.setState(ClimbState.DISABLED);
      }

        if (RobotContainer.driverR1.getAsBoolean()) {
          climb.set(0.6);
        } else if (RobotContainer.PitPad.rightBumper().getAsBoolean()) {
          climb.set(-.4);
        } 
        else if(RobotContainer.driverL1.getAsBoolean()) {
          PDH.setSwitchableChannel(true);
        }
        else {
          climb.set(0);
          
            if (RobotContainer.climb.getClimbPosition() <= -0.17 && RobotContainer.climb.getClimbPosition() >= -0.16) {
              System.out.println("Hey I should stop");
          
              RobotContainer.climb.setState(ClimbState.DISABLED);
            }
          
      }
      break;
    }
  }
}

/*
 * 
 * 
 * public getClimb
 * \
 * public getSetClimb
 * 
 * setClimb []
 * 
 * 
 * 
 * 
 * 
 */
