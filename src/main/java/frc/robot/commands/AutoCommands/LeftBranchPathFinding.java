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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LimelightHelpers;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants.POSES;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftBranchPathFinding extends Command {


        boolean isFinished = false;


    private Pose2d Targetpose = null;

  public LeftBranchPathFinding() {

  }

  final public InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

  final public JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);

  // Called when the command is initially scheduled.
  @Override
 public void initialize() {
        
            System.out.println("rightBranchPathfinding method called.");
        
            double aprilTagID = LimelightHelpers.getFiducialID("limelight");
            // Blue paths

            switch ((int) aprilTagID) {
            case 17:
                 {
                    Targetpose = POSES.REEF_C;
                }
                break;

            case 18 :
            {
                Targetpose = POSES.REEF_A;
            }
                break;
            

            case 19:
            {
                 Targetpose = POSES.REEF_K;
            }
                break;

            case 20:
            {
                 Targetpose = POSES.REEF_I;
            }
                break;

            case 21:
            {
                 Targetpose = POSES.REEF_G;
            }
                break;

            case 22:
            {
                 Targetpose = POSES.REEF_E;
            }
                break;

            // Red Paths
            case 6:
            {
                 Targetpose = POSES.REEF_K;
            }
                break;

            case 7:
            {
                 Targetpose = POSES.REEF_A;
            }
                break;

            case 8:
            {
                 Targetpose = POSES.REEF_C;
            }
                break;

            case 9:
            {
                 Targetpose = POSES.REEF_E;
            }
                break;

            case 10:
            {
                 Targetpose = POSES.REEF_G;
            }
                break;

            case 11:
                 {
                 Targetpose = POSES.REEF_I;
               
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
        
                    1,
        
                    .5,
        
                    .5,
        
                    .5
        
            );

        

    if (Targetpose != null) {


        Command followLeftPath = AutoBuilder.pathfindToPose(
            Targetpose,
            constraints,
            0.03
    );
            followLeftPath.schedule();

        
    }

   
    
    
}



    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
    
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (driverL2.getAsBoolean() == false) {
        isFinished = true;
    }

    if (isFinished == true){  
        return true ;
    }

    else{
        return false;
    }

}
}