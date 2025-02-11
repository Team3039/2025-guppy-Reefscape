// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class CoolElevator  extends TimedRobot {
  private final TalonFX m_fx = new TalonFX(12, "MainCan");

  private final XboxController m_joystick = new XboxController(1);


  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public CoolElevator() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));



    configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    m_fx.setPosition(0);
  }

  @Override
  public void robotPeriodic() {
    /* Report the current position */
    System.out.println("Position: " + m_fx.getPosition());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double L1 = m_joystick.getLeftY() * 10; // Go for plus/minus 10 rotations
    if (Math.abs(L1) <= 0.1) { // Joystick deadzone
      L1 = 1;

      double L2 = m_joystick.getLeftY() * 10; //
    if (Math.abs(L2) <= 0.1) { 
      L2 = 4;

      double zero = m_joystick.getLeftY() * 10; 
    if (Math.abs(zero) <= 0.1) { 
      zero = 0;
      
    }

    if (m_joystick.getLeftBumperButton()) {
      /* Use position voltage */
      m_fx.setControl(m_positionVoltage.withPosition(L1));
    } else if (m_joystick.getRightBumperButton()) {
      /* Use position torque */
      m_fx.setControl(m_positionTorque.withPosition(L1));
    } else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
    }
    //
    if (m_joystick.getRightBumperButton()) {
      /* Use position voltage */
      m_fx.setControl(m_positionVoltage.withPosition(L2));
    } else if (m_joystick.getRightBumperButton()) {
      /* Use position torque */
      m_fx.setControl(m_positionTorque.withPosition(L2));
    } else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
    }
    if (m_joystick.getAButton()) {
      /* Use position voltage */
      m_fx.setControl(m_positionVoltage.withPosition(zero));
    } else if (m_joystick.getAButton()) {
      /* Use position torque */
      m_fx.setControl(m_positionTorque.withPosition(zero));
    } else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
    }

  }
  }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

 
}