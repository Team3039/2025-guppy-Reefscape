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
public class RightBranchPathfinding extends Command {
    
    private PathPlannerPath Path = null;
    
        /** Creates a new LeftBranchPathFinding. */
        public RightBranchPathfinding() {
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        
            System.out.println("rightBranchPathfinding method called.");
        
            double aprilTagID = LimelightHelpers.getFiducialID("limelight");
            // Blue paths

            switch ((int) aprilTagID) {
            case 17:
                try {
                    Path = PathPlannerPath.fromPathFile("Blue D");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 18:
                try {
                    Path = PathPlannerPath.fromPathFile("Blue B");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 19:
                try {
                    Path = PathPlannerPath.fromPathFile("Blue L");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 20:
                try {
                    Path = PathPlannerPath.fromPathFile("Blue J");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 21:
                try {
                System.out.println("hey Miles your really cool");
                Path = PathPlannerPath.fromPathFile("Blue H");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 22:
                try {
                System.out.println("im weird ");
                Path = PathPlannerPath.fromPathFile("Blue F");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            // Red Paths
            case 6:
                try {
                    Path = PathPlannerPath.fromPathFile("RED L");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 7:
                try {
                    Path = PathPlannerPath.fromPathFile("RED B");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 8:
                try {
                    Path = PathPlannerPath.fromPathFile("RED D");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 9:
                try {
                    Path = PathPlannerPath.fromPathFile("RED F");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 10:
                try {
                    Path = PathPlannerPath.fromPathFile("RED H");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            case 11:
                try {
                Path = PathPlannerPath.fromPathFile("RED J");
                } catch (IOException | ParseException e) {
                e.printStackTrace();
                }
                break;

            default:

                break;
            }
           
            return;
        
        }
        
          
          @Override
          public void execute() {
        
        
            PathConstraints constraints = new PathConstraints(
        
                    .5,
        
                    .5,
        
                    .5,
        
                    .5
        
            );
        

    if (Path != null) {

        System.out.println("if you see this I work but right");

        Command followRightPath = AutoBuilder.pathfindThenFollowPath(Path, constraints);

        followRightPath.schedule();
        
        
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

//     final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

//     final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);


// if (driverR2.getAsBoolean()) {
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
