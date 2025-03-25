
package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTargetingResults;

public class Limelight extends SubsystemBase {
    // private SwerveDrivePoseEstimator m_poseEstimator;
    CommandSwerveDrivetrain drivetrain;
    Alliance alliance;

    //  private static final RectanglePoseArea field =
        // new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

    // Define the RectanglePoseArea class
    // private static class RectanglePoseArea {
    //     // private final Translation2d bottomLeft;
    //     // private final Translation2d topRight;

    //     // public RectanglePoseArea(Translation2d bottomLeft, Translation2d topRight) {
    //     //     this.bottomLeft = bottomLeft;
    //     //     this.topRight = topRight;
    //     // }

    //     // public boolean isPoseWithinArea(Pose2d pose) {
    //     //     double x = pose.getX();
    //     //     double y = pose.getY();
    //     //     return x >= bottomLeft.getX() && x <= topRight.getX()
    //     //         && y >= bottomLeft.getY() && y <= topRight.getY();
    //     // }
    // }


    private String ll = "limelight";
    private Boolean enable = true;
    private Boolean trust = false;
    private int fieldError = 0;
    private int distanceError = 0;
    private Pose2d botpose;
    
    private PathConstraints constraints = new PathConstraints(

            .5,

            .5,

            .5,

            .5

    );

    

    /** Creates a new Limelight. */
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);
    }

     @Override
  public void periodic() {}
    // if (enable) {
//       Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
//       Double confidence = 1 - ((targetDistance - 1) / 6);
//       LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(ll);
//       if (results.targetingResults.validTarget) {
//         botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
//         if (field.isPoseWithinArea(botpose)) {
//           if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
//               || trust
//               || ((results.targetingResults.targets_Fiducials instanceof java.util.Collection) 
//                   && ((java.util.Collection<?>) results.targetingResults.targets_Fiducials).size() > 1)) {
                
//             drivetrain.addVisionMeasurement(
//                 botpose,
//                 Timer.getFPGATimestamp()
//                     - (results.targetingResults.targetX / 1000.0)
//                     - (results.targetingResults.targetY / 1000.0),
//                 VecBuilder.fill(confidence, confidence, .01));
//           } else {
//             distanceError++;
//             SmartDashboard.putNumber("Limelight Error", distanceError);
//           }
//         } else {
//           fieldError++;
//           SmartDashboard.putNumber("Field Error", fieldError);
//         }
//       }
//     }
//   }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public void trustLL(boolean trust) {
        this.trust = trust;
    }

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

            Command followLeftPath = AutoBuilder.pathfindThenFollowPath(leftPath, constraints);

            AutoBuilder.followPath(leftPath).schedule();
            
        
            return followLeftPath;

        }
        return new Command() {
        };

    }

    public Command rightBranchPathfinding() {
        // B
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