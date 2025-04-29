// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LimelightHelpers;
import frc.robot.controllers.InterpolatedPS4Gamepad;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightBranchPathfinding extends Command {
  /** Creates a new LeftBranchPathFinding. */
  public RightBranchPathfinding() {

        

        
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

 PathConstraints constraints = new PathConstraints(

            .5,

            .5,

            .5,

            .5

    );

    System.out.println("rightBranchPathfinding method called.");
    // L
    // J
    // H
    // F
    // D

    // Blue paths

    double aprilTagID = LimelightHelpers.getFiducialID("limelight");

    PathPlannerPath path = null;

    // Blue paths

    switch ((int) aprilTagID) {
        case 17:
            try {
                path = PathPlannerPath.fromPathFile("Blue D");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }

        case 18:
            try {
                path = PathPlannerPath.fromPathFile("Blue B");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }

        case 19:
            try {
                path = PathPlannerPath.fromPathFile("Blue L");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 20:
            try {
                path = PathPlannerPath.fromPathFile("Blue J");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 21:
            try {

                System.out.println("hey Miles your really cool");

                path = PathPlannerPath.fromPathFile("Blue H");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 22:
            try {

                System.out.println("im weird ");

                path = PathPlannerPath.fromPathFile("Blue F");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;
        // Red Paths

        case 6:
            try {
                path = PathPlannerPath.fromPathFile("RED L");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 7:
            try {
                path = PathPlannerPath.fromPathFile("RED B");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 8:
            try {
                path = PathPlannerPath.fromPathFile("RED D");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 9:
            try {
                path = PathPlannerPath.fromPathFile("RED F");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 10:
            try {
                path = PathPlannerPath.fromPathFile("RED H");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        case 11:
            try {
                path = PathPlannerPath.fromPathFile("RED J");
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                break;
            }
            break;

        default:
            // handle other cases
            break;
    }
    if (path != null) {

        System.out.println("if you see this I work but right");

        Command followRightPath = AutoBuilder.pathfindThenFollowPath(path, constraints);

        followRightPath.schedule();
        
        
    }




}

  
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);


if (driverR2.getAsBoolean()) {
    interrupted = false;
}

else {
    interrupted = true;}

  }




  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
