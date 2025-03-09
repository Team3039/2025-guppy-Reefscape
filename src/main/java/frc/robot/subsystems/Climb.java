// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Climb extends SubsystemBase {

  // Create the possible states of the climb
  public enum ClimbState {
    DISABLED,
    CLIMBING
  }

  // Create a variable to store the current state of the climb
  public ClimbState climbState = ClimbState.DISABLED;

  // Create a talonfx for the climb
  TalonFX climb = new TalonFX(TunerConstants.CLIMB);

  // Climb Constructor
  public Climb() {}

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



  @Override
  public void periodic() {
    
    // Climb State Machine
    switch (climbState) {

      // In the disabled state, the climb is disabled
      case DISABLED:
        climb.disable();
        break;
      case CLIMBING:
        if (RobotContainer.driverPad.leftBumper().getAsBoolean()) {
          climb.set(0.5);
        } else if (RobotContainer.driverPad.rightBumper().getAsBoolean()) {
          climb.set(-0.5);
        } else {
          climb.set(0);
        }
        break;
    }
  }

  public void getClimbPosition() {
    climb.getPosition().getValueAsDouble();
  }
}
