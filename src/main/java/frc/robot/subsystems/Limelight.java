
package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Util.RectanglePoseArea;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    Alliance alliance;
    private String ll = "limelight";
    private Boolean enable = true;
    private Boolean trust = false;
    private int fieldError = 0;
    private int distanceError = 0;
    private Pose2d botpose;
    // private static final RectanglePoseArea field =
    // new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54,
    // 8.02));

    /** Creates a new Limelight. */
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);
    }

    @Override
    public void periodic() {
        // Add your periodic code here

        SmartDashboard.putNumber("Fiducial ID", LimelightHelpers.getFiducialID("limelight"));

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

    public Command leftBranchAlign() {
        PathPlannerPath leftPath = null;

        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                degreesToRadians(360), degreesToRadians(540));

        double aprilTagID = LimelightHelpers.getFiducialID("limelight");

        // 17 18 19 20 21 22
        // c a k I g e

        // Blue paths
        switch ((int) aprilTagID) {
            case 17:
                try {
                    leftPath = PathPlannerPath.fromPathFile("Blue C");
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }
                break;

            case 18:
                try {
                    leftPath = PathPlannerPath.fromPathFile("Blue A");
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

        if (leftPath != null) {
            Command leftBranchCommand = AutoBuilder.pathfindThenFollowPath(
                    leftPath,
                    constraints

            );

            return leftBranchCommand;
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
                    path = PathPlannerPath.fromPathFile("Blue H");
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                    break;
                }
                break;

            case 22:
                try {
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

            Command rightSideCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints);

            return rightSideCommand;
        }

        return new Command() {
        };

    }

    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Math.toRadians(360), Math.toRadians(540));
}