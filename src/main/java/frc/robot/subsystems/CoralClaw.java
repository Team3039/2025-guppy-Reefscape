// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class CoralClaw extends SubsystemBase {


  // Create the possible states of the claw
  public enum ClawState {
    IDLE,
    PASSIVE,
    CORAL,
    RELEASE
    
  }

  // Create a variable to store the current state of the claw
  ClawState clawState = ClawState.IDLE;

  // Keep track of whether or not the intake has a coral
  public boolean hasCoral = false;

  // Create a talonfx for the claw
  TalonFX claw = new TalonFX(TunerConstants.CLAW);

  // This CANrange is used to detect coral in the intake
  CANrange coralCANRange = new CANrange(TunerConstants.CORALCANRANGE);

  // This CANrange is used to align with the branch when scoring coral
  // CANrange branchCANRange = new CANrange(TunerConstants.BRANCHCANRANGE);

  // Claw Constructor
  public CoralClaw() {

    // Create a talonfx configurator.
    TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    // Inverted and Neutral Modes
    clawConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    clawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the claw motor
    claw.getConfigurator().apply(clawConfig);

  }
    // Create a CANrange configurator

  /**
   * Get the current state of the claw
   * 
   * @return the current state of the claw as a ClawState
   */
  public ClawState getState() {
    return clawState;
  }
  
  /**
   * Set the state of the claw
   * 
   * @param state the state to set the claw to
   */
  public void setState(ClawState state) {
    clawState = state;
  }

  /**
   * Set the speed of the claw
   * <p>
   * Positive values will intake algae, negative values will intake coral
   * 
   * @param speed the speed to set the claw to (-1 to 1)
   */
  public void setWheelSpeed(double speed) {
    claw.set(speed);
  }

  

  /** 
   * Check to see whether the intake has either gamepiece
   * 
   * @return true if the intake has either gamepiece, false otherwise
   */
  public boolean hasGamepiece() {
    return hasCoral;
  }


  public boolean isCoralIn() {
    return coralCANRange.getDistance().getValueAsDouble() < 0.15 ;
  }

  /**
   * Check to see if the claw is aligned with the branch.
   * It does this by checking the distance detected by the branchCANRange.
   * If it detects an object closer than 0.5 meters, it is likely the branch, and thus we are aligned.
   * 
   * @return true if the claw is aligned with the branch, false otherwise
   */






  @Override
  public void periodic() {
    // SmartDashboard.putNumber("CanRange Distance Detected", branchCANRange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Claw Current", claw.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Claw Status", String.valueOf(claw.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putBoolean("Has Coral", isCoralIn());
    

    // If the robot is ready to score a coral, rumble the driver controller to indicate this
    // if (isBranchDetected() && Elevator.getSetpoint() > 8) {
    //   RobotContainer.operatorPad.setRumble(RumbleType.kBothRumble, 5);
    // }
    // else {
    //   RobotContainer.operatorPad.setRumble(RumbleType.kBothRumble, 0); 
    // }

    // Claw State Machine
    switch (clawState) {

      // In the idle state, the claw does not intake, and it isnt deactivated
      case IDLE:
        setWheelSpeed(0);
        hasCoral = false;
        break;





      // In the coral state, the claw will spin in reverse to intake coral,
      //  deactivating if the coralCANRange detects an object
      case CORAL:
        if (isCoralIn() ) {
          Timer.delay(.20);
          setWheelSpeed(0);
          hasCoral = true;
        }
        else if (!hasGamepiece()) {
          setWheelSpeed(0.3);
        }
        break;

      // In the algae state, the claw will spin forwards to intake algae, 
      //  deactivating if the current exceeds 10 amps

      // In the release state, the claw will spin forwards to release the gamepiece
      //  and will release the deactivation lock
      case RELEASE:
        setWheelSpeed(0.3);
        hasCoral = false;
        break;



      // In the passive state, the claw will not intake, and will deactivate the intake. 
      //  This will be used when the claw has a gamepiece
      case PASSIVE:
        setWheelSpeed(-.3);
        

        break;
    }
  


    

  }
}