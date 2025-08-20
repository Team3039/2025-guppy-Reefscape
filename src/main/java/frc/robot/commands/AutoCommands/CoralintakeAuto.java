// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;
import frc.robot.commands.ActuateElevatorToIdle;
import frc.robot.commands.ActuateWristToIdle;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw.ClawState;





import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralintakeAuto extends Command {
  /** Creates a new CoralintakeAuto. */
  public CoralintakeAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.claw.setState(ClawState.CORAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.claw.hasGamepiece();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.err.println("We all done here");

    RobotContainer.claw.setState(ClawState.IDLE);


    this.cancel();
  }


}
