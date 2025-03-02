//nothing to see here


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

    public TalonFX elavator = new TalonFX(TunerConstants.ELEVATOR);

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			TunerConstants.ElevatorPID.ELEVATOR_KS,
			TunerConstants.ElevatorPID.ELEVATOR_KG,
			TunerConstants.ElevatorPID.ELEVATOR_KV);

	private PIDController controller = new PIDController(
        TunerConstants.ElevatorPID.ELEVATOR_KP,
        TunerConstants.ElevatorPID.ELEVATOR_KI,
        TunerConstants.ElevatorPID.ELEVATOR_KD);

	
	// kraken encoder ticks
	public static double setpointElevator = 0;

	public Elevator() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;	
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;	

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		elavator.getConfigurator().apply(config);
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
        elavator.set(percent);
    }

    public void setElevatorClosedLoop() {
        double output = 0;
        output = controller.calculate(elavator.getPosition().getValueAsDouble(), setpointElevator) + TunerConstants.ElevatorPID.ELEVATOR_KS;
        elavator.set(MathUtil.clamp(output, -.3, .3));
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setElevatorSetpoint(double setpoint) {
        setpointElevator = setpoint;
    }

    public boolean isAtSetpoint(double tolerance) {
        return Math.abs((setpointElevator - getElevatorPosition())) <= tolerance;
    }

    public double getElevatorPosition() {
        return elavator.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", getElevatorPosition());
		SmartDashboard.putNumber("Elevator Output", RobotContainer.driverPad.getLeftY());
		// SmartDashboard.putString("Elevator State", String.valueOf(getState()));

		// SmartDashboard.putNumber("Elevator Output", elevator.get());
		// System.out.println(encoder.getPosition());
		// System.out.println(elevator.get());
		// System.out.println(isAtSetpoint(false));
		switch (elevatorState) {
			case IDLE:
				setElevatorSetpoint(0);
				// setElevatorClosedLoop();
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.driverPad.getLeftY());
				break;
			case POSITION:
				// setElevatorClosedLoop();
				break;
		}
	}
}


