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
import frc.robot.LimelightHelpers;
import frc.robot.controllers.InterpolatedPS4Gamepad;

import frc.robot.generated.TunerConstants.POSES;
// import miracle;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightBranchPathfinding extends Command {
    
 

    private Pose2d Targetpose = null;

    private boolean finish = false;
    
        /** Creates a new LeftBranchPathFinding. */
        public RightBranchPathfinding() {
        }
    final public InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

    final public JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
 
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        
            System.out.println("rightBranchPathfinding method called.");
        
            double aprilTagID = LimelightHelpers.getFiducialID("limelight");
            // Blue paths

            switch ((int) aprilTagID) {
            case 17:
                 {
                    Targetpose = POSES.REEF_D;
                }
                break;

            case 18 :
            {
                Targetpose = POSES.REEF_B;
            }
                break;
            

            case 19:
            {
                 Targetpose = POSES.REEF_L;
            }
                break;

            case 20:
            {
                 Targetpose = POSES.REEF_J;
            }
                break;

            case 21:
            {
                 Targetpose = POSES.REEF_H;
            }
                break;

            case 22:
            {
                 Targetpose = POSES.REEF_F;
            }
                break;

            // Red Paths
            case 6:
            {
                 Targetpose = POSES.REEF_L;
            }
                break;

            case 7:
            {
                 Targetpose = POSES.REEF_B;
            }
                break;

            case 8:
            {
                 Targetpose = POSES.REEF_D;
            }
                break;

            case 9:
            {
                 Targetpose = POSES.REEF_F;
            }
                break;

            case 10:
            {
                 Targetpose = POSES.REEF_H;
            }
                break;

            case 11:
                 {
                 Targetpose = POSES.REEF_J;
               
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


        Command followRightPath = AutoBuilder.pathfindToPose(
            Targetpose,
            constraints,
            0.03 );
            
        followRightPath.schedule();
    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    
    
   cancel();

    }


  








  // Returns true when the command should end.
  @Override
  
    public boolean isFinished() {



       
    if (driverR2.getAsBoolean() == false) {
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