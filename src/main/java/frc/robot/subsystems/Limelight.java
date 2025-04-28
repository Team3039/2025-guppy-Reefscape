
package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
// import frc.robot.LimelightHelpers.LimelightTargetingResults;

public class Limelight extends SubsystemBase {
    
    private PathConstraints constraints = new PathConstraints(

            .5,

            .5,

            .5,

            .5

    );

    

    public Command leftBranchDriveTo() {
        PathPlannerPath leftPath = null;

        double aprilTagID = LimelightHelpers.getFiducialID("limelight");

        // 17 18 19 20 21 22
        // c a k I g e

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

                 System.out.println("Hey I work");
                break;

            case 22:
                try {
                    leftPath = PathPlannerPath.fromPathFile("Blue E");
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }

                 System.out.println("Hey Im a dumb ass");

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

        if (leftPath != null) {          
        
            
            System.out.println("if you see this I work but right");

            Command leftBranchDriveTo = AutoBuilder.pathfindThenFollowPath(leftPath, constraints);

            AutoBuilder.followPath(leftPath).schedule();
            
        
            return leftBranchDriveTo;

        }
        return new Command() {
        };

    }

    public Command rightBranchPathfinding() {      

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

            AutoBuilder.followPath(path).schedule();
            
            return followRightPath;
        }



        return new Command() {
        };

    }

}

