// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.





package frc.robot.commands;


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StopPathFinding extends Command {

     public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();




  public StopPathFinding() {
    addRequirements(RobotContainer.drivetrain);

  }



  @Override
  public void initialize() {

    drivetrain.applyRequest(() -> brake);
  }

  @Override
  public void execute() {
    drivetrain.applyRequest(() -> brake);


  }

  @Override
  public void end(boolean interrupted) {
  }

    @Override
    public boolean isFinished() {

          return false;
  
    }
  }
