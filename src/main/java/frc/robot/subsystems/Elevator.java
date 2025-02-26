//nothing to see here


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

    public TalonSRX elavator = new TalonSRX(TunerConstants.ELEVATOR);


	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			TunerConstants.ElevatorPID.ELEVATOR_KS,
			TunerConstants.ElevatorPID.ELEVATOR_KG,
			TunerConstants.ElevatorPID.ELEVATOR_KV);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
        TunerConstants.ElevatorPID.ELEVATOR_KP,
        TunerConstants.ElevatorPID.ELEVATOR_KI,
			TunerConstants.ElevatorPID.ELEVATOR_KD,
			new TrapezoidProfile.Constraints(
                TunerConstants.ElevatorPID.ELEVATOR_MAX_VEL,
                TunerConstants.ElevatorPID.ELEVATOR_MAX_ACCEL));

	private PIDController controller = new PIDController(
        TunerConstants.ElevatorPID.ELEVATOR_KP,
        TunerConstants.ElevatorPID.ELEVATOR_KI,
        TunerConstants.ElevatorPID.ELEVATOR_KD);

	
	// neo rotations
	public static double setpointElevator = 0;

	public Elevator() {
        // elavator.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        // elavator.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        elavator.setNeutralMode(NeutralMode.Brake);

        elavator.setInverted(false);

        elavator.configForwardSoftLimitEnable(true);
        elavator.configReverseSoftLimitEnable(true);
        elavator.configForwardSoftLimitThreshold(88);
        elavator.configReverseSoftLimitThreshold(0);


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
        elavator.set(ControlMode.PercentOutput, percent);
    }

    public void setElevatorClosedLoop(boolean isProfiled) {
        double output = 0;
        if (isProfiled) {
            profiledController.setGoal(setpointElevator);
            output = profiledController.calculate(elavator.getSelectedSensorPosition()) +
                    feedForward.calculate(profiledController.getSetpoint().velocity);
            elavator.set(ControlMode.Position, setpointElevator, DemandType.ArbitraryFeedForward, output);
        } else {
            output = controller.calculate(elavator.getSelectedSensorPosition(), setpointElevator) + TunerConstants.ElevatorPID.ELEVATOR_KS;
            elavator.set(ControlMode.Position, setpointElevator, DemandType.ArbitraryFeedForward, MathUtil.clamp(output, -.75, .85));
		}
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
        setpointElevator = setpoint;
    }

    public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
        return Math.abs((setpointElevator - elavator.getSelectedSensorPosition())) <= tolerance;
    }

    public double getPosition() {
        return elavator.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
		// SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
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
				setElevatorOpenLoop(RobotContainer.driverPad.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop(false);
				break;
		}
	}
}








//I see you are looking at the elevator subsystem.
// This is a very important subsystem that is used to move the elevator up and down.






// I should do this about now