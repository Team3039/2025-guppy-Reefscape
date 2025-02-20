// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import org.ejml.equation.ManagerFunctions.Input1;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;



public class Claw extends SubsystemBase {

	public enum ClawState {
		IDLE,
		PASSIVE,
		INTAKE,
		RELEASE
	}




	// public Spark motor = new Spark(0);
	// public Spark motorTwo = new Spark(1);

	public ClawState clawState = ClawState.IDLE;

	
	public TalonFX claw = new TalonFX(TunerConstants.CLAW);
	public TalonFX wrist = new TalonFX(TunerConstants.WRIST);
	DigitalInput input = new DigitalInput(0);
	DutyCycleEncoder WristEncoder = new DutyCycleEncoder(1);
	public TalonFX shoulder = new TalonFX(TunerConstants.SHOULDER);
	DigitalInput bazinga = new DigitalInput(1);
	DutyCycleEncoder ShoulderEncoder = new DutyCycleEncoder(1);
	
	public Timer timer = new Timer();



	public boolean deactivateIntake = false;

	public boolean allowSnapping = false;

	public Claw() {
		// pH.disableCompressor();
		// leftWheels.setIdleMode(IdleMode.kBrake);
		// rightWheels.setIdleMode(IdleMode.kBrake);
		claw.setNeutralMode(NeutralModeValue.Brake);

		// leftWheels.setInverted(false);
		// rightWheels.setInverted(true);

		timer.reset();
	}

	public void setState(ClawState state) {
		clawState = state;
	}

	public ClawState getState() {
		return clawState;
	}

	public void setWheelSpeed(double speed) {
		claw.setControl(new VelocityVoltage(speed));
	}


	public boolean isIntakeDeactivated() {
		return deactivateIntake;
	}

	

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Is Intake Deactivated", isIntakeDeactivated());
		SmartDashboard.putString("Claw State", String.valueOf(getState()));
		SmartDashboard.putNumber("Wrist pose", WristEncoder.get());
		SmartDashboard.putNumber("Shoulder pose", ShoulderEncoder.get());


		switch (clawState) {
			case IDLE:
				timer.stop();
				timer.reset();
				setWheelSpeed(0);
				deactivateIntake = false;
				break;
			case PASSIVE:
				deactivateIntake = true;
				setWheelSpeed(0.08);
				break;
			case INTAKE:
				
			 
				if (!deactivateIntake) {
					setWheelSpeed(0.8);
				}
				if (deactivateIntake && timer.get() > 0.1) {
					setWheelSpeed(0);
					wrist.setControl(new VelocityVoltage(20));
					timer.stop();
					timer.reset();
				}
				break;
			case RELEASE:
				deactivateIntake = false;
				if (DriverStation.isAutonomousEnabled())  {
					setWheelSpeed(-0.8);
				}
				else {
					setWheelSpeed(-0.8);
				}
		}
	}
}