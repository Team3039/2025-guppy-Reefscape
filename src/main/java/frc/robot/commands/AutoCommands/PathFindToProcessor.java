// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants.POSES;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFindToProcessor extends Command {

  Pose2d Targetpose = POSES.PROCESSOR;

    
        /** Creates a new LeftBranchPathFinding. */
        public PathFindToProcessor() {
        }
     final public InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

  final public JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);

  private boolean finish = false;

        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          PathConstraints constraints = new PathConstraints(
        
          1,

          .5,

          1,

          1

  );




Command followRightPath = AutoBuilder.pathfindToPose(
  Targetpose,
  constraints,
  0.0
);
followRightPath.schedule();


}

          
          @Override
          public void execute() {
      
    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

}




  // Returns true when the command should end.
  @Override
  
    public boolean isFinished() {

       if (driverSquare.getAsBoolean() == false) {
        finish = true;
    }

    if (finish == true){  
        return true ;
    }

    else{
        return false;
    }
    }
}
  

