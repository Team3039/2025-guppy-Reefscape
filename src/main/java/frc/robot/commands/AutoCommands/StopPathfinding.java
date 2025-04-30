// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LimelightHelpers;
import frc.robot.controllers.InterpolatedPS4Gamepad;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopPathfinding extends Command {
    
        /** Creates a new LeftBranchPathFinding. */
        public StopPathfinding() {
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {

            PathPlannerAuto.setCurrentTrajectory(null);

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


            return false;
    }
}
  

