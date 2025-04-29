// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LimelightHelpers;
import frc.robot.controllers.InterpolatedPS4Gamepad;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftBranchPathFinding extends Command {


    private PathPlannerPath leftPath = null;

  public LeftBranchPathFinding() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


// 17 18 19 20 21 22
        // c a k I g e
        double aprilTagID = LimelightHelpers.getFiducialID("limelight");


        // Blue paths
        switch ((int) aprilTagID) {
          case 17:

              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue K");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }
              break;

          case 18:

              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue K");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }
              break;

          case 19:
              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue K");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }
              break;

          case 20:
              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue I");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }
              break;

          case 21:
              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue G");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }

              break;

          case 22:
              try {
                  leftPath = PathPlannerPath.fromPathFile("Blue E");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }

              break;

          // RED paths
          case 6:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED K");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
              }
              break;

          case 7:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED A");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
                  break;
              }
          case 8:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED C");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
                  break;
              }
              break;

          case 9:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED E");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
                  break;
              }
              break;

          case 10:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED G");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
                  break;
              }
              break;

          case 11:
              try {
                  leftPath = PathPlannerPath.fromPathFile("RED I");
              } catch (IOException | ParseException e) {
                  e.printStackTrace();
                  break;
              }
              break;

          default:
              // handle other cases
              break;
      }
  }

  @Override
  public void execute() {
    
    
        PathConstraints constraints = new PathConstraints(
    
                .5,
    
                .5,
    
                .5,
    
                .5
    
        );
    

if (leftPath != null) {


    Command followRightPath = AutoBuilder.pathfindThenFollowPath( leftPath, constraints);

    followRightPath.schedule();
    
    
}

}

    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

//     final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

//     final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);


// if (driverL2.getAsBoolean()) {
//     interrupted = false;

// }

// else {
//     interrupted = true;}

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        
    

    return false;
  }
}
