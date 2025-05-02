// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import static frc.robot.generated.TunerConstants.WonderOnOverToConstants.kMaxSpeed;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants.WonderOnOverToConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Autonomous alignment command using vision feedback:
 * - Uses PID control for X and Y positioning
 * - Aligns to either left or right vision target
 * - Uses Limelight TX/TY values for position feedback
 */
public class WonderOnOverTo extends Command {
  private final PIDController distanceController = new PIDController(0.0369, 0., 0.0013);
  private final PIDController lateralController = new PIDController(0.01, 0., 0.0003);
  private final PIDController thetaController = new PIDController(0.25, 0., 0.0);

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private final boolean aligningL;

  private boolean latAligned;

  // Constants for alignment targets in meters
  private static final double LEFT_DISTANCE_METERS = 3.9;
  private static final double RIGHT_DISTANCE_METERS = 1.25;
  private static final double LEFT_LATERAL_METERS = 1.15;
  private static final double RIGHT_LATERAL_METERS = -1.25;
  private static final double LATERAL_ALIGNMENT_THRESHOLD = 4.0;

  public WonderOnOverTo(CommandSwerveDrivetrain drivetrain, boolean aligningL) {
    this.drivetrain = drivetrain;
    this.aligningL = aligningL;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(0.5);
    distanceController.setTolerance(0.5 * WonderOnOverToConstants.INCHES_TO_METERS);
    lateralController.setTolerance(0.5 * WonderOnOverToConstants.INCHES_TO_METERS);
    latAligned = false;

    drivetrain.turnOffAutoScore();
  }

  @Override
  public void execute() {
    double rot = getRot();

    // Set rotation target based on current angle zone
    if (rot < 30 && rot > -30) {
      thetaController.setSetpoint(0.);
    } else if (rot < 90 && rot > 30) {
      thetaController.setSetpoint(60.);
    } else if (rot < 150 && rot > 90) {
      thetaController.setSetpoint(120.);
    } else if (Math.abs(rot) > 150) {
      thetaController.setSetpoint(180.);
    } else if (rot < -90 && rot > -150) {
      thetaController.setSetpoint(-120.);
    } else {
      thetaController.setSetpoint(-60);
    }

    // Set X (forward/backward) and Y (side-to-side) setpoints
    distanceController.setSetpoint(aligningL ? LEFT_DISTANCE_METERS : RIGHT_DISTANCE_METERS);
    lateralController.setSetpoint(aligningL ? LEFT_LATERAL_METERS : RIGHT_LATERAL_METERS);

    // Get Limelight measurements
    double yError = drivetrain.getTXLeft();
    double xError = drivetrain.getTYLeft();

    // PID outputs
    double rotOutput = thetaController.calculate(rot);
    double xOutput = -kMaxSpeed * distanceController.calculate(xError);
    double yOutput = -kMaxSpeed * lateralController.calculate(yError);

    // Clamp speeds for safety
    xOutput = MathUtil.clamp(xOutput, -kMaxSpeed, kMaxSpeed);
    yOutput = MathUtil.clamp(yOutput, -kMaxSpeed, kMaxSpeed);

    if (DriverStation.isTeleop()) {
      if (Math.abs(lateralController.getError()) < LATERAL_ALIGNMENT_THRESHOLD && !latAligned) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(yOutput).withRotationalRate(rotOutput));
      } else {
        latAligned = true;
        drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rotOutput));
      }
    } else {
      drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rotOutput));
    }

    drivetrain.setAligning(true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    drivetrain.setAligning(false);
  }

  @Override
  public boolean isFinished() {
    return !drivetrain.getTVLeft()
        || Math.abs(distanceController.getSetpoint() - getMeasurement()) < (DriverStation.isAutonomous() ? 2.0 : 1.0)
        || distanceController.atSetpoint();
  }

  public double getMeasurement() {
    return drivetrain.getTYLeft();
  }

  public double getRot() {
    return drivetrain.getPose().getRotation().getDegrees();
  }
}
