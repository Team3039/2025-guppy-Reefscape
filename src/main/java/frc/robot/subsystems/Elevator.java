package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ClimbState {
		IDLE,
		MANUAL,
		POSITION,
		CLIMB_UP,
	}

	public ClimbState climbState = ClimbState.IDLE;

	public TalonFX climbA = new TalonFX(TunerConstants.CBT);
	// public CANSparkMax climbB = new CANSparkMax(Constants.Ports.CLIMB_B,
	// MotorType.kBrushless);


	private PIDController controller = new PIDController(
			TunerConstants.cbt.CLIMB_KP,
			TunerConstants.cbt.CLIMB_KI,
			TunerConstants.cbt.CLIMB_KD);

	// kraken rotations
	public static double setpointClimb = 0;

	public Elevator() {
		
		controller.setTolerance(3);
	}

	public void setState(ClimbState state) {
		climbState = state;
	}

	public ClimbState getState() {
		return climbState;
	}

	public void setClimbOpenLoop(double percent) {
		climbA.set(percent);
	}

	public void setClimbClosedLoop() {
		double output = 0;

		output = controller.calculate(encoder.getPosition(), setpointClimb) + TunerConstants.cbt.CLIMB_KS;
		climbA.set(MathUtil.clamp(output, -1, 1));
	}

	public static double getSetpoint() {
		return setpointClimb;
	}

	public static void setSetpoint(double setpoint) {
		setpointClimb = setpoint;
	}

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointClimb - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Climb Current Draw", climbA.getOutputCurrent());
		// SmartDashboard.putNumber("Climb Encoder", encoder.getPosition());
		// SmartDashboard.putString("Climb State", String.valueOf(getState()));
		// SmartDashboard.putNumber("Climb Output", climbA.get());
		// SmartDashboard.putNumber("Setpoint Climb", getSetpoint());
		switch (climbState) {
			case IDLE:
				if (RobotContainer.driverPad.leftTrigger().getAsBoolean()){
					setClimbOpenLoop(-.9);
				}
				if (RobotContainer.driverPad.leftBumper().getAsBoolean()){
					setClimbOpenLoop(.9);
				}
			
				break;
			case MANUAL:
			
				setClimbOpenLoop(-1 * RobotContainer.driverPad.getLeftY());// intuitive
				
				break;
			case POSITION:
				setClimbClosedLoop();
				break;
			case CLIMB_UP:
				setSetpoint(16);
				setClimbClosedLoop();
				break;
		}
	}
}


