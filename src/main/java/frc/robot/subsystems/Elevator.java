//nothing to see here

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;


public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION,
	}

	private ElevatorState elevatorState = ElevatorState.IDLE;

	private final TalonFX elevator = new TalonFX(TunerConstants.ELEVATOR);
	private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

	private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
	public static double setpointElevator = 0;

	public Elevator() {
		// TalonFX configuration
		TalonFXConfiguration config = new TalonFXConfiguration();

		// Current limits
		config.CurrentLimits.SupplyCurrentLimit = 20;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 120;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		// Soft limits (adjust thresholds as needed)
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -46;

		// Brake mode
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Apply base config
		elevator.getConfigurator().apply(config);

		// Motion Magic settings
		motionMagicConfigs.MotionMagicCruiseVelocity = 80;
		motionMagicConfigs.MotionMagicAcceleration = 80;

		// PID + feedforward constants (Slot 0)
		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kP = TunerConstants.Elevator.ELEVATOR_KP;
		slot0Configs.kI = TunerConstants.Elevator.ELEVATOR_KI;
		slot0Configs.kD = TunerConstants.Elevator.ELEVATOR_KD;
		slot0Configs.kS = TunerConstants.Elevator.ELEVATOR_KS;

		// Apply configs
		elevator.getConfigurator().apply(motionMagicConfigs);
		elevator.getConfigurator().apply(slot0Configs);
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	/** Use Motion Magic to move to the setpoint. */
	public void setElevatorPosition() {
		motionMagicControl
			.withPosition(setpointElevator * -1) // Adjust sign as needed
			.withFeedForward(TunerConstants.Elevator.ELEVATOR_KS);
		elevator.setControl(motionMagicControl);
	}

	/** Manual percent output control, includes feedforward. */
	public void setElevatorPercent(double percent) {
		elevator.set(percent + TunerConstants.Elevator.ELEVATOR_KS);
	}

	public double getElevatorPosition() {
		return elevator.getPosition().getValueAsDouble() * -1; // Reverse if needed
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs(setpointElevator - getElevatorPosition()) <= tolerance;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", getElevatorPosition());
		SmartDashboard.putNumber("Elevator Output", elevator.get());
		SmartDashboard.putNumber("Elevator Setpoint", getSetpoint());

		switch (elevatorState) {
			case IDLE:
				setElevatorPosition(); 
				break;

			case MANUAL:
				setElevatorPercent(RobotContainer.driverPad.getLeftY() * 0.3);
				break;

			case POSITION:
				setElevatorPosition();
				break;
		}
	}
}