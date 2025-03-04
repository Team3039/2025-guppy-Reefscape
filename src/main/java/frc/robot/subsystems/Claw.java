// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Claw extends SubsystemBase {

  public enum ClawState {
    IDLE,
    PASSIVE,
    CORAL,
    ALGAE,
    RELEASE
  }

  ClawState clawState = ClawState.IDLE;

  public boolean deactivateIntake = false;

  TalonFX claw = new TalonFX(TunerConstants.CLAW);

  CANrange canRange = new CANrange(TunerConstants.CANrange);

  public Claw() {
    TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    clawConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    clawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    claw.getConfigurator().apply(clawConfig);

    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

    // canRangeConfig.ProximityParams.ProximityThreshold = 0.08;
    // canRangeConfig.ProximityParams.ProximityHysteresis = 0.02;

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
    SmartDashboard.putNumber("CanRange Distance Detected", canRange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Claw Current", claw.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Claw State", String.valueOf(getState()));


    switch (clawState) {
      case IDLE:
        setWheelSpeed(0);
        deactivateIntake = false;
        break;
      case CORAL:
        if (canRange.getDistance().getValueAsDouble() < 0.08) {
          setWheelSpeed(0);
          deactivateIntake = true;
        }
        else if (!deactivateIntake) {
          setWheelSpeed(-0.3);
        }
        break;
      case ALGAE:
        if (claw.getSupplyCurrent().getValueAsDouble() > 10) {
          setWheelSpeed(0);
          deactivateIntake = true;
        }
        else if (!deactivateIntake) {
          setWheelSpeed(0.3);
        }
        break;
      case RELEASE:
        setWheelSpeed(0.7);
        deactivateIntake = false;
        break;
      case PASSIVE:
        setWheelSpeed(0);
        deactivateIntake = true;
        break;
    }
  


  }
}
