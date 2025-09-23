//nothing to see here

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Elevator extends SubsystemBase {

	// Create the possible states of the elevator
	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION,
	}



	// Create a variable to store the current state of the elevator
	public ElevatorState elevatorState = ElevatorState.IDLE;

	// Create a talonfx for the elevator
	public TalonFX elavator = new TalonFX(TunerConstants.ELEVATOR);

	// Create a PID Controller for the elevator
	private PIDController controller = new PIDController(
			TunerConstants.Elevator.ELEVATOR_KP,
			TunerConstants.Elevator.ELEVATOR_KI,
			TunerConstants.Elevator.ELEVATOR_KD);




	// Create a variable to store the setpoint of the elevator in kraken encoder
	// ticks
	public static double setpointElevator = 0;

	// Elevator Constructor
	public Elevator() {

		// Create a talonfx configurator.
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.SupplyCurrentLimit = 20;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;

		config.CurrentLimits.StatorCurrentLimit = 120;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		// Soft Limits
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -46;

		// Inverted and Neutral Modes
		// config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Apply the configurator to the elevator motor
		elavator.getConfigurator().apply(config);
	}

	/**
	 * Get the state of the elevator
	 * 
	 * @return The state of the elevator as an ElevatorState
	 */
	public ElevatorState getState() {
		return elevatorState;
	}

	/**
	 * Set the state of the elevator
	 * 
	 * @param state The state to set the elevator to
	 */
	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	/**
	 * Set the position of the elevator to setpointElevator using PID and
	 * Feedforward.
	 * <p>
	 * It will first calculate the pid output, clamping it between -.3 and .3.
	 * <p>
	 * Then, it adds the KS (feedforward) constant to the output.
	 * <p>
	 * This result is the percent output that will be assigned to the elevator
	 */


	public void setElevatorPosition() {
		double output = 0;


		


		if(elavator.getPosition().getValueAsDouble() < 8){
			output = MathUtil.clamp(controller.calculate(elavator.getPosition().getValueAsDouble(), setpointElevator * -1),
					-.18, .25) +
					TunerConstants.Elevator.ELEVATOR_KS;
	}
	
	if(elavator.getPosition().getValueAsDouble() > 8){
		output = MathUtil.clamp(controller.calculate(elavator.getPosition().getValueAsDouble(), setpointElevator * -1),
				-.15, .25) +
				TunerConstants.Elevator.ELEVATOR_KS;
	}


		elavator.set(output);

	// public void setElevatorPosition() {

	// 	// Ensure motionMagicControl is properly initialized
	// 	motionMagicControl = new MotionMagicVoltage(.2)
	// 		.withPosition(setpointElevator * -1)
	// 		.withFeedForward(TunerConstants.Elevator.ELEVATOR_KS);

	// 	// Ensure the motor controller is set to Motion Magic control
	// 	elavator.setControl(motionMagicControl);
	}

	/**
	 * Set the output of the elevator with feedforward
	 * 
	 * @param percent The percentage to set the elevator to
	 */
	public void setElevatorPercent(double percent) {
		elavator.set(percent + TunerConstants.Elevator.ELEVATOR_KS);
	}

	/**
	 * Get the current position of the elevator
	 * 
	 * @return the current angle of the wrist in kraken ticks
	 */
	public double getElevatorPosition() {
		return elavator.getPosition().getValueAsDouble() * -1;
	}

	/**
	 * Get the current setpoint of the elevator
	 * 
	 * @return the current setpoint of the elevator
	 */
	public static double getSetpoint() {
		return setpointElevator;
	}

	/**
	 * Set the setpoint of the elevator
	 * 
	 * @param setpoint the setpoint to set the elevator to, in kraken encoder ticks
	 */
	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	/**
	 * Check if the elevator is at the setpoint within a given tolerance
	 * 
	 * @param tolerance the tolerance to check if the elevator is at the setpoint
	 * @return true if the wrist is at the setpoint within the tolerance, false
	 *         otherwise
	 */
	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointElevator - getElevatorPosition())) <= tolerance;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", getElevatorPosition());
		SmartDashboard.putNumber("Elevator Output", elavator.get());
		// SmartDashboard.putNumber("Elevator Output Current",
		// elavator.getSupply$Current().getValueAsDouble());
		// SmartDashboard.putString("Elevator State", String.valueOf(getState()));
		SmartDashboard.putNumber("Elevator Setpoint", getSetpoint());

		// Elevator State Machine
		switch (elevatorState) {

			// In the Idle state, the elevator rests at the bottom of the robot
			case IDLE:
					setElevatorPosition();
					
				break;

			// In the Manual state, the elevator is controlled directly by the operator
			case MANUAL:
				setElevatorPercent(RobotContainer.driverPad.getLeftY() * 0.3);
				break;

			// In the Position state, the elevator is controlled by the setpoint
			case POSITION:
				setElevatorPosition();
				break;
		}
	}
}