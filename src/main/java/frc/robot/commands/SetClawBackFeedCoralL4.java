// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw.ClawState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClawBackFeedCoralL4 extends Command {

  boolean wedonehere = false;

  public SetClawBackFeedCoralL4() {
    addRequirements(RobotContainer.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.claw.setState(ClawState.BACKFEED);
    Timer.delay(0.2);
      RobotContainer.claw.setState(ClawState.IDLE);
      wedonehere = true;
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (wedonehere == true) { interrupted = true;
    }
      RobotContainer.claw.setState(ClawState.IDLE);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
