
package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Util.RectanglePoseArea;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
//   private static final RectanglePoseArea field =
//         new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
    }
  


  @Override
  public void periodic() {
    


}
      


  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  
  }

    public void leftBranchPathfinding() {
        

          try {
            // //Blue paths
            // PathPlannerPath BlueA = PathPlannerPath.fromPathFile("Blue A");
            // PathPlannerPath Bluek = PathPlannerPath.fromPathFile("Blue k");
            // PathPlannerPath BlueI = PathPlannerPath.fromPathFile("Blue I");
            // PathPlannerPath BlueG = PathPlannerPath.fromPathFile("Blue G");
            // PathPlannerPath BlueE = PathPlannerPath.fromPathFile("Blue E");
            // PathPlannerPath BlueC = PathPlannerPath.fromPathFile("Blue C");

            // //Red paths
            // PathPlannerPath REDA = PathPlannerPath.fromPathFile("RED A");
            // PathPlannerPath REDk = PathPlannerPath.fromPathFile("RED k");
            // PathPlannerPath REDI = PathPlannerPath.fromPathFile("RED I");
            // PathPlannerPath REDG = PathPlannerPath.fromPathFile("RED G");
            // PathPlannerPath REDE = PathPlannerPath.fromPathFile("RED E");
            // PathPlannerPath REDC = PathPlannerPath.fromPathFile("RED C");


            public Command leftBranchAlign() {
                PathPlannerPath path;
        
                PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        
                double aprilTagID = LimelightHelpers.getFiducialID("Limelight"); 
        
            
                switch (aprilTagID) {
                    case 5:
                        // load the tag 5 left branch path
                    case 7:
                        // load the tag 7 left branch path
                }
            
        
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints
                );
        
                return pathfindingComand;
            }


           
        } catch (FileVersionException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
            } catch (ParseException e) {
                e.printStackTrace();
            }
        }


        public void rightBranchPathfinding() {

            try {

                // B
                // L
                // J
                // H
                // F
                // D



              //Blue paths
            // PathPlannerPath BlueB = PathPlannerPath.fromPathFile("Blue B");
            // PathPlannerPath BlueL = PathPlannerPath.fromPathFile("Blue L");
            // PathPlannerPath BlueJ = PathPlannerPath.fromPathFile("Blue J");
            // PathPlannerPath BlueH = PathPlannerPath.fromPathFile("Blue H");
            // PathPlannerPath BlueF = PathPlannerPath.fromPathFile("Blue F");
            // PathPlannerPath BlueD = PathPlannerPath.fromPathFile("Blue D");

            // //Red paths
            // PathPlannerPath REDB = PathPlannerPath.fromPathFile("RED B");
            // PathPlannerPath REDL = PathPlannerPath.fromPathFile("RED L");
            // PathPlannerPath REDJ = PathPlannerPath.fromPathFile("RED J");
            // PathPlannerPath REDH = PathPlannerPath.fromPathFile("RED H");
            // PathPlannerPath REDF = PathPlannerPath.fromPathFile("RED F");
            // PathPlannerPath REDD = PathPlannerPath.fromPathFile("RED D");
  
              PathConstraints constraints = new PathConstraints(
                  3.0, 4.0,
                  Math.toRadians(360), Math.toRadians(540));
                 
                  Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                      path,
                      constraints);
  
          } catch (FileVersionException e) {
              e.printStackTrace();
          } catch (IOException e) {
              e.printStackTrace();
          } catch (ParseException e) {
              e.printStackTrace();
          }
  

}
}