// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;

public class Wrist extends SubsystemBase {

  public enum WristState {
    IDLE,
    MANUAL,
    POSITION,
    PASSIVE
  }

  public WristState wristState = WristState.IDLE;

  public TalonSRX wrist = new TalonSRX(TunerConstants.WRIST);

  public ArmFeedforward feedForward = new ArmFeedforward(
    TunerConstants.Wrist.WRIST_KS,
    TunerConstants.Wrist.WRIST_KG,
    TunerConstants.Wrist.WRIST_KV);

  public ProfiledPIDController profiledController = new ProfiledPIDController(
    TunerConstants.Wrist.WRIST_KP,
      TunerConstants.Wrist.WRIST_KI,
      TunerConstants.Wrist.WRIST_KD,
      new TrapezoidProfile.Constraints(
        TunerConstants.Wrist.WRIST_MAX_VEL,
        TunerConstants.Wrist.WRIST_MAX_ACCEL));

  public PIDController controller = new PIDController(
    TunerConstants.Wrist.WRIST_KP,
      TunerConstants.Wrist.WRIST_KI,
      TunerConstants.Wrist.WRIST_KD);

  public static double setpointWrist = 0;
  public static double idleSetpoint = 0;

  double wristSetpointOffset = 0;

  public Wrist() {
    // wrist.configFactoryDefault();%
    wrist.setNeutralMode(NeutralMode.Brake);

    // Wrist must start in the vertical position in order to be legal. DONT FORGET TO DO THIS PLS
    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // wrist.setInverted(true);
    // wrist.setSensorPhase(true);
    
    wrist.configForwardSoftLimitEnable(true);
    wrist.configReverseSoftLimitEnable(true);
    wrist.configForwardSoftLimitThreshold(degreesToTicks(121));
    wrist.configReverseSoftLimitThreshold(degreesToTicks(-15));

    wrist.setSelectedSensorPosition(degreesToTicks(120.9));
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

  public void setWristPosition(boolean isProfiled) {
    if (isProfiled) {
      profiledController.setGoal(setpointWrist);
      // wrist.set(ControlMode.Position, degreesToTicks(degrees));
      wrist.set(ControlMode.PercentOutput, profiledController.calculate(wrist.getSelectedSensorPosition()) +
          feedForward.calculate(Math.toRadians(profiledController.getSetpoint().position),
              profiledController.getSetpoint().velocity));
    } else {
      wrist.set(ControlMode.PercentOutput, MathUtil.clamp(controller.calculate(
          ticksToDegrees(wrist.getSelectedSensorPosition()),
          setpointWrist), -.25, .3),
          DemandType.ArbitraryFeedForward,
          (Math.cos(Math.toRadians(ticksToDegrees(wrist.getSelectedSensorPosition()))) * TunerConstants.Wrist.WRIST_KG +
          TunerConstants.Wrist.WRIST_KS));
    }
  }

  public void setWristPercent(double percent) {
    wrist.set(ControlMode.PercentOutput, percent +
        Math.cos(Math.toRadians(ticksToDegrees(wrist.getSelectedSensorPosition()))) * TunerConstants.Wrist.WRIST_KG +
        TunerConstants.Wrist.WRIST_KS);
  }

  public static double getSetpoint() {
    return setpointWrist;
  }

  public static void setSetpoint(double setpoint) {
    setpointWrist = setpoint;
  }

  public double getWristPosition() {
    return ticksToDegrees(wrist.getSelectedSensorPosition());
  }

	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
		return Math.abs((setpointWrist - ticksToDegrees(wrist.getSelectedSensorPosition()))) <= tolerance;
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
    

    
  }
}