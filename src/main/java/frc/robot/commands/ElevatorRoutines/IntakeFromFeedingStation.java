// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateElevatorToIdle;
import frc.robot.commands.ActuateWristToIdle;
import frc.robot.commands.SetClawIntakeCoral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromFeedingStation extends SequentialCommandGroup {
  /** Creates a new IntakeFromFeedingStation. */
  public IntakeFromFeedingStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActuateElevatorToIdle(10),
      new ActuateWristToIdle(10),
      new SetClawIntakeCoral()
    );
  }
}