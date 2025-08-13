
package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class RightBranchPathfinding extends Command {

  

  public CommandSwerveDrivetrain m_drivetrain;


  public void Stop(CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);

}


  APTarget TargetPose = null;
 
    
    APResult out;

    
  
    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); /* tune this for your robot! */
  
  
    // public LeftBranchPathFinding(APTarget target, CommandSwerveDrivetrain drivetrain) {
    //   // m_target = target;
    //   m_drivetrain = drivetrain;
    //   addRequirements(drivetrain);
    // }

    public RightBranchPathfinding(CommandSwerveDrivetrain drivetrain) {
      m_drivetrain = drivetrain;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {

    System.out.println("RightBranchPathFinding initialize called.");

            int aprilTagID = (int) LimelightHelpers.getFiducialID("limelight");


    
            switch (aprilTagID) {
                // Blue alliance tags
                case 17: TargetPose = new APTarget(TunerConstants.POSES.REEF_D); break;
                case 18: TargetPose = new APTarget(TunerConstants.POSES.REEF_B); break;
                case 19: TargetPose = new APTarget(TunerConstants.POSES.REEF_L); break;
                case 20: TargetPose = new APTarget(TunerConstants.POSES.REEF_J); break;
                case 21: TargetPose = new APTarget(TunerConstants.POSES.REEF_H); break;
                case 22: TargetPose = new APTarget(TunerConstants.POSES.REEF_F);break;
                
                // Red alliance tags
                case 6: TargetPose = new APTarget(TunerConstants.POSES.REEF_L); break;
                case 7: TargetPose = new APTarget(TunerConstants.POSES.REEF_B); break;
                case 8: TargetPose = new APTarget(TunerConstants.POSES.REEF_D); break;
                case 9: TargetPose = new APTarget(TunerConstants.POSES.REEF_F); break;
                case 10: TargetPose = new APTarget(TunerConstants.POSES.REEF_H); break;
                case 11: TargetPose = new APTarget(TunerConstants.POSES.REEF_J); break;
    
                // If no valid tag is seen 
                default: System.out.println("No AprilTag seen dip dumb");
                  this.cancel(); break;
                  
            }
    
        
        }

  
    @Override
    public void execute() {
      ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getStateCopy().Speeds;
      
      Pose2d pose = m_drivetrain.getPose();

      out = TunerConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, TargetPose);
  
      m_drivetrain.setControl(m_request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle()));

          System.out.println("Im going right :D");
    }
  
    @Override
    public boolean isFinished() {

      System.out.println("Im checking if im done going right >:-(");

    return TunerConstants.kAutopilot.atTarget(m_drivetrain.getPose(), TargetPose);

  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("im done going right :p");

    m_drivetrain.setControl(m_request
    .withVelocityX(0)
    .withVelocityY(0)
    .withTargetDirection(out.targetAngle()));


    Stop(m_drivetrain);
  }
  }