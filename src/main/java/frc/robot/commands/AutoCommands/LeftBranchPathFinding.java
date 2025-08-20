package frc.robot.commands.AutoCommands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.FlippingUtil;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class LeftBranchPathFinding extends Command {

  public void Stop(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

}

  

  
  APTarget TargetPose = null;
 
   public CommandSwerveDrivetrain m_drivetrain;
   
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

    public LeftBranchPathFinding(CommandSwerveDrivetrain drivetrain) {
      m_drivetrain = drivetrain;
      addRequirements(drivetrain);
    }


  
    @Override
    public void initialize() {

      
                 


    System.out.println("LeftBranchPathFinding initialize called.");

            int aprilTagID = (int) LimelightHelpers.getFiducialID("limelight");
    
            switch (aprilTagID) {
                // Blue alliance tags
                case 17: TargetPose = new APTarget(TunerConstants.POSES.REEF_C); break;
                case 18: TargetPose = new APTarget(TunerConstants.POSES.REEF_A); break;
                case 19: TargetPose = new APTarget(TunerConstants.POSES.REEF_K); break;
                case 20: TargetPose = new APTarget(TunerConstants.POSES.REEF_I); break;
                case 21: TargetPose = new APTarget(TunerConstants.POSES.REEF_G); break;
                case 22: TargetPose = new APTarget(TunerConstants.POSES.REEF_E); break;
                


                // Red alliance tags
                case 6: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_K)); break;
                case 7: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_A)); break;
                case 8: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_C)); break;
                case 9: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_E)); break;
                case 10: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_G)); break;
                case 11: TargetPose = new APTarget(FlippingUtil.flipFieldPose(TunerConstants.POSES.REEF_I)); break;
    
                // If no valid tag is seen 
                default: System.out.println("No AprilTag seen dip dumb");
                    this.cancel(); break;
                  
            }
    
        
        }

  
    @Override
    public void execute() {



      Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {

                ;
            }
          }

      

      ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getStateCopy().Speeds;
      
      Pose2d pose = m_drivetrain.getPose();

      out = TunerConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, TargetPose);
  
      m_drivetrain.setControl(m_request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle()));
        

          // System.out.println("Im going left :D");
    }
  
    @Override
    public boolean isFinished() {

        // System.out.println("Im checking if im done going left >:-(");

        // System.out.println(m_drivetrain.getPose());

        System.out.println(TunerConstants.kAutopilot.atTarget(m_drivetrain.getPose(), TargetPose));


      return TunerConstants.kAutopilot.atTarget(m_drivetrain.getPose(), TargetPose);

    }
  
    @Override
    public void end(boolean interrupted) {
     System.out.println("im done going left :p");
 
      m_drivetrain.getModulePositions();

      m_drivetrain.setControl(m_request
    .withVelocityX(0)
    .withVelocityY(0));

      this.cancel();
      Stop(m_drivetrain);
    }
  }


