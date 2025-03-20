// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathFinding;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rightBranchAlign extends Command {
  /** Creates a new rightBranchAlign. */
private final Limelight limelight;

public rightBranchAlign(Limelight limelight) {
    this.limelight = limelight;
}


// Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // System.out.println("But he made graduation");
// 6 million what???

    limelight.rightBranchPathfinding();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
