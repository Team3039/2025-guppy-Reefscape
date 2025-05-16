// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Wrist.WristState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimbToSetpoints extends Command {
boolean areyoureadykids = false;

  /** Creates a new SetElevatorManualOverride. */
  public SetClimbToSetpoints() {
    addRequirements(RobotContainer.climb);

    if (RobotContainer.climb.getClimbPosition() >= 0.47) {
      RobotContainer.climb.setState(ClimbState.DISABLED);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(areyoureadykids == false){

            RobotContainer.climb.setState(ClimbState.GetReadyTOClimb);
 
            areyoureadykids = true;
    }

    if(areyoureadykids == true){

            RobotContainer.climb.setState(ClimbState.CLIMBING);
     }
    

  }


  public void periodic() {
    if (RobotContainer.climb.getClimbPosition() <= 0.47) {
            

      RobotContainer.climb.setState(ClimbState.DISABLED);
    }
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climb.setState(ClimbState.DISABLED);
    RobotContainer.wrist.setState(WristState.IDLE);

   
    }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
