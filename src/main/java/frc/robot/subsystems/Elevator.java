// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

	public CANSparkMax elevator = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);

	public RelativeEncoder encoder = elevator.getEncoder();

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			Constants.Elevator.ELEVATOR_KS,
			Constants.Elevator.ELEVATOR_KG,
			Constants.Elevator.ELEVATOR_KV);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD,
			new TrapezoidProfile.Constraints(
					Constants.Elevator.ELEVATOR_MAX_VEL,
					Constants.Elevator.ELEVATOR_MAX_ACCEL));

	private PIDController controller = new PIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD);

	SparkMaxLimitSwitch forwardLimit = elevator.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
	SparkMaxLimitSwitch ReverseLimit = elevator.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

	// neo rotations
	public static double setpointElevator = 0;

	public Elevator() {
		forwardLimit.enableLimitSwitch(false);
		ReverseLimit.enableLimitSwitch(false);

		elevator.setIdleMode(IdleMode.kBrake);

		elevator.setInverted(false);

		elevator.enableSoftLimit(SoftLimitDirection.kForward, true);
		elevator.enableSoftLimit(SoftLimitDirection.kReverse, true);
		elevator.setSoftLimit(SoftLimitDirection.kForward, 88);
		elevator.setSoftLimit(SoftLimitDirection.kReverse, 0);

		// elevatorA.burnFlash();
		elevator.burnFlash();

		// elevator.setStatusFramePeriod();

		controller.setTolerance(3);
		profiledController.setTolerance(3);
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
		elevator.set(percent);
	}

	public void setElevatorClosedLoop(boolean isProfiled) {
		double output = 0;
		if (isProfiled) {
			profiledController.setGoal(setpointElevator);
			output = profiledController.calculate(encoder.getPosition()) +
					feedForward.calculate(profiledController.getSetpoint().velocity);
			elevator.set(output);
		} else {
			output = controller.calculate(encoder.getPosition(), setpointElevator) + Constants.Elevator.ELEVATOR_KS;
			elevator.set(MathUtil.clamp(output, -.75, .85));
			// elevator.set(MathUtil.clamp(output, -.2, .3));

		}
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
		return Math.abs((setpointElevator - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
		// SmartDashboard.putString("Elevator State", String.valueOf(getState()));

		// SmartDashboard.putNumber("Elevator Output", elevator.get());
		// System.out.println(encoder.getPosition());
		// System.out.println(elevator.get());
		// System.out.println(isAtSetpoint(false));
		switch (elevatorState) {
			case IDLE:
				setSetpoint(0);
				setElevatorClosedLoop(false);
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.operatorPad.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop(false);
				break;
		}
	}
}