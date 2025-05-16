// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateElevatorToSetpoint;
import frc.robot.commands.ActuateWristToSetpoint;
import frc.robot.commands.SetClawBackFeedCoralL2;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralL2Auto extends SequentialCommandGroup {
  /** Creates a new ScoreCoralL2. */
  public ScoreCoralL2Auto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
      new ActuateWristToSetpoint(280, 2),
      new ActuateElevatorToSetpoint( 7, 3),
      new SetClawBackFeedCoralL2()
    );
  }
}
