// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Claw extends SubsystemBase {

  public enum ClawState {
		IDLE,
		PASSIVE,
		INTAKE,
		RELEASE
	}

  ClawState clawState = ClawState.IDLE;

  public boolean deactivateIntake = false; 

  TalonFX claw = new TalonFX(TunerConstants.CLAW);

  CANrange proximitySensor = new CANrange(TunerConstants.CANrange);

  public Claw() {
    TalonFXConfiguration config = new TalonFXConfiguration();

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		claw.getConfigurator().apply(config);
  }

  public void setState(ClawState state) {
		clawState = state;
	}

	public ClawState getState() {
		return clawState;
	}

	public void setWheelSpeed(double speed) {
		claw.set(speed);
	}

	public boolean isIntakeDeactivated() {
		return deactivateIntake;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
