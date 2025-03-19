// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * The Wrist subsystem is responsible for controlling the wrist of the robot.
 */
public class Wrist extends SubsystemBase {

  // Create the possible states of the wrist
  public enum WristState {
    IDLE,
    MANUAL,
    POSITION,
    CLIMB,
    PASSIVE
  }

  // Create a variable to store the current state of the wrist
  public WristState wristState = WristState.IDLE;

  // Create a talonfx for the wrist
  public TalonFX wrist = new TalonFX(TunerConstants.WRIST);

  // Create an absolute encoder for the wrist
  public DutyCycleEncoder wristEncoder = new DutyCycleEncoder(TunerConstants.WRIST_ENCODER);

  // Create a PID Controller for the wrist
  public PIDController controller = new PIDController(
    TunerConstants.Wrist.WRIST_KP,
      TunerConstants.Wrist.WRIST_KI,
      TunerConstants.Wrist.WRIST_KD);

  // Create a variable to store the setpoint of the wrist
  public static double setpointWrist = 0;

  // Create a variable to store the idle setpoint of the wrist
  public static double idleSetpoint = 3;

  // Wrist Constructor
  public Wrist() {

    // Create a talonfx configurator.
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Soft Limits
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 135;	
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 150;	

    // Inverted and Neutral Mode
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply Configs to the talonfx on the wrist
		wrist.getConfigurator().apply(config);

    // Have Wrist Encoder Output in degrees
  }

  /** Obtain the current state of the wrist 
   * 
   * @return the current state of the wrist as a WristState
  */
  public WristState getState() {
    return wristState;
  }

  /** Set the state of the wrist
   * 
   * @param state the state to set the wrist to
  */
  public void setState(WristState state) {
    wristState = state;
  }


  /**
   * Set the position of the wrist to setpointWrist using PID and Feedforward.
   * <p>
   * It will first calculate the pid output, clamping it between -.2 and .2.
   * <p>
   * For the gravity feedforward, it multiplies the KG constant by the cosine of the wrist angle.
   * <p>
   * Finally, it adds the KS constant to the output.
   * <p>
   * This result is the percent output that will be assigned to the wrist
   */
  public void setWristPosition() {
    double output = 0;
    output = MathUtil.clamp(controller.calculate(getWristPosition(), setpointWrist), -.2, .2);
    setWristPercent(output*-1);
  }

  /**
   * Set the percent output of the wrist talonfx with feedforward.
   * <p>
   * It will first set the percent output to the given percent.
   * <p>
   * For the gravity feedforward, it multiplies the KG constant by the cosine of the wrist angle.
   * <p>
   * Finally, it will add the KS constant to the output.
   * <p>
   * This result is the percent output that will be assigned to the wrist
   * 
   * @param percent the percent output to set the wrist to
   */
  public void setWristPercent(double percent) {
    double output = 0;


    


    output = percent +
    Math.cos(Math.toRadians(getWristPosition() + TunerConstants.Wrist.WRIST_COG_OFFSET )) * TunerConstants.Wrist.Coral_WRIST_KG +
    TunerConstants.Wrist.Coral_WRIST_KS;

    if(getWristPosition() > 51 && output < 0 ) output = 0 ;

    
    if(getWristPosition() < -10 && output > 0 ) output = 0 ;

    wrist.set(output) ;


// if(RobotContainer.claw.hasCoral)



  // wrist.set(percent +
  //       Math.cos(Math.toRadians(getWristPosition() + TunerConstants.Wrist.WRIST_COG_OFFSET )) * TunerConstants.Wrist.Coral_WRIST_KG +
  //       TunerConstants.Wrist.Coral_WRIST_KS );}
// }
// else{
//     // wrist.set(percent +
//     //      Math.cos(Math.toRadians(getWristPosition()  + TunerConstants.Wrist.WRIST_COG_OFFSET)) * TunerConstants.Wrist.WRIST_KG +
//     //     TunerConstants.Wrist.WRIST_KS );}
  }

  /**
   * Get the current setpoint of the wrist
   * 
   * @return the current setpoint of the wrist
   */
  public static double getSetpoint() {
    return setpointWrist;
  }

  /**
   * Set the setpoint of the wrist
   * 
   * @param setpoint the setpoint to set the wrist to
   */
  public static void setSetpoint(double setpoint) {
    setpointWrist = setpoint;
  }

  /**
   * Get the current position of the wrist, accounting for the offset of the rev encoder
   * 
   * @return the current angle of the wrist in degrees
   */
  public double getWristPosition() {
    return (wristEncoder.get() * 360) + TunerConstants.Wrist.WRIST_OFFSET;
  }

  /**
   * Check if the wrist is at the setpoint within a given tolerance
   * heres the thing
   * @param tolerance the tolerance to check if the wrist is at the setpoint
   * @return true if the wrist is at the setpoint within the tolerance, false otherwise
   */
	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointWrist - getWristPosition())) <= tolerance;
	}
//the thing goes thing a thing thing
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getWristPosition());
    SmartDashboard.putString("Wrist State", String.valueOf(getState()));
    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint());
    SmartDashboard.putNumber("Wrist Output", wrist.get());
    
    idleSetpoint = getState().equals(WristState.IDLE) ? idleSetpoint : setpointWrist;
    
    // Wrist State Machine
    switch (wristState) {
      
      // In the idle state, the wrist rests within the robot
      case IDLE:
        setSetpoint(50);
         setWristPosition();
        break;

      // In the manual state, the wrist is controlled directly by the operator
      case MANUAL:
        setWristPercent(RobotContainer.operatorPad.getRightY() * 0.4);
        break;
        
      case PASSIVE:
        break;

      // In the position state, the wrist is controlled by the setpoint
      case POSITION:
         setWristPosition();
        break;
      default:
        break;

        case CLIMB:
        setSetpoint(45);
        setWristPosition();
        break;
        
      
    }
    
  }
}