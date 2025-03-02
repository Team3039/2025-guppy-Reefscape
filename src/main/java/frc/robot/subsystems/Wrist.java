// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Wrist extends SubsystemBase {

  public enum WristState {
    IDLE,
    MANUAL,
    POSITION,
    PASSIVE
  }

  public WristState wristState = WristState.IDLE;

  public TalonFX wrist = new TalonFX(TunerConstants.WRIST);

  public PIDController controller = new PIDController(
    TunerConstants.WristPID.WRIST_KP,
      TunerConstants.WristPID.WRIST_KI,
      TunerConstants.WristPID.WRIST_KD);

  public static double setpointWrist = 0;
  public static double idleSetpoint = 3;

  double wristSetpointOffset = 0;

  public Wrist() {
    TalonFXConfiguration config = new TalonFXConfiguration();

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;	
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;	

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		wrist.getConfigurator().apply(config);
  }

  public WristState getState() {
    return wristState;
  }

  public void setState(WristState state) {
    wristState = state;
  }

  public double degreesToTicks(double degrees) {
    double armRotations = degrees / 360;
    double ticks = armRotations * 4096;
    return ticks;
  }

  // give the encoder value to get degrees
  public double ticksToDegrees(double ticks) {
    double armRotations = ticks / 4096;
    double armDegrees = armRotations * 360;
    return armDegrees;
  }

  public void setWristPosition() {
    wrist.set(MathUtil.clamp(controller.calculate(getWristPosition(), setpointWrist), -.2, .2) + 
        (Math.cos(Math.toRadians(getWristPosition()))) * TunerConstants.WristPID.WRIST_KG +
        TunerConstants.WristPID.WRIST_KS);
  }

  public void setWristPercent(double percent) {
    wrist.set(percent +
        Math.cos(Math.toRadians(ticksToDegrees(getWristPosition()))) * TunerConstants.WristPID.WRIST_KG +
        TunerConstants.WristPID.WRIST_KS);
  }

  public static double getSetpoint() {
    return setpointWrist;
  }

  public static void setSetpoint(double setpoint) {
    setpointWrist = setpoint;
  }

  public double getWristPosition() {
    return ticksToDegrees(wrist.getPosition().getValueAsDouble());
  }

	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
		return Math.abs((setpointWrist - ticksToDegrees(getWristPosition()))) <= tolerance;
	}

  public double getWristOffset() {
    return wristSetpointOffset;
  }

  public void changeWristOffset(double offset) {
    wristSetpointOffset += offset;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Wrist Absolute Encoder",
    // wrist.getSelectedSensorPosition());
    // System.out.println(ticksToDegrees(wrist.getSelectedSensorPosition()));
    // System.out.println(wrist.getMotorOutputPercent());
    // SmartDashboard.putNumber("Wrist Currnent Input", wrist.getSupplyCurrent());
    // SmartDashboard.putNumber("Wrist Current Output", wrist.getStatorCurrent());
    // System.out.println(setpointWrist);
    SmartDashboard.putNumber("Wrist Angle", getWristPosition());
    // SmartDashboard.putString("Wrist State", String.valueOf(getState()));
    // SmartDashboard.putNumber("Wrist Offset", getWristOffset());
    // SmartDashboard.putNumber("Wrist Setpoint", getSetpoint());
    idleSetpoint = getState().equals(WristState.IDLE) ? idleSetpoint : setpointWrist;
    
    switch (wristState) {
      case IDLE:
        setSetpoint(idleSetpoint);
        setWristPosition();
        break;
      case MANUAL:
        setWristPercent(RobotContainer.operatorPad.getRightY());
        break;
      case PASSIVE:
        break;
      case POSITION:
        setWristPosition();
        break;
      default:
        break;
      
    }
    
  }
}