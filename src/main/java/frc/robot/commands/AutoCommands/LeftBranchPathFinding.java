package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class LeftBranchPathFinding extends Command {
    private final APTarget m_target;
    private final CommandSwerveDrivetrain m_drivetrain;
    APResult out;

  
    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(1, 0, 0); /* tune this for your robot! */
  
  
    public LeftBranchPathFinding(APTarget target, CommandSwerveDrivetrain drivetrain) {
      m_target = target;
      m_drivetrain = drivetrain;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
        System.out.println(":D :D :D");
    }
  
    @Override
    public void execute() {
      ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getStateCopy().Speeds;

      // Translation2d velocity = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond); 
      
      Pose2d pose = m_drivetrain.getPose();

      out = TunerConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, m_target);
  
      m_drivetrain.setControl(m_request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle()));

          System.out.println("Im going :D");
    }
  
    @Override
    public boolean isFinished() {

        System.out.println("Im done :D");

      return TunerConstants.kAutopilot.atTarget(m_drivetrain.getPose(), m_target);

    }
  
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(m_request
      .withVelocityX(0)
      .withVelocityY(0)
      .withTargetDirection(out.targetAngle()));
    }
  }



// package frc.robot.commands.AutoCommands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.LimelightHelpers;
// import frc.robot.controllers.InterpolatedPS4Gamepad;
// import frc.robot.generated.TunerConstants.POSES;

// public class LeftBranchPathFinding extends Command {

//     private Pose2d targetPose = null;
//     private boolean isFinished = false;
//     private Command followLeftPath = null;

//     private final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0);
//     private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);

//     @Override
//     public void initialize() {
//         System.out.println("LeftBranchPathFinding initialize called.");

//         int aprilTagID = (int) LimelightHelpers.getFiducialID("limelight");

//         switch (aprilTagID) {
//             // Blue alliance tags
//             case 17 -> targetPose = POSES.REEF_C;
//             case 18 -> targetPose = POSES.REEF_A;
//             case 19 -> targetPose = POSES.REEF_K;
//             case 20 -> targetPose = POSES.REEF_I;
//             case 21 -> targetPose = POSES.REEF_G;
//             case 22 -> targetPose = POSES.REEF_E;

//             // Red alliance tags
//             case 6 -> targetPose = POSES.REEF_K;
//             case 7 -> targetPose = POSES.REEF_A;
//             case 8 -> targetPose = POSES.REEF_C;
//             case 9 -> targetPose = POSES.REEF_E;
//             case 10 -> targetPose = POSES.REEF_G;
//             case 11 -> targetPose = POSES.REEF_I;

//             // If no valid tag is seen 
//             default -> {
//                 System.out.println("No AprilTag seen dip shit");
//                 // targetPose = POSES.REEF_A;
//             }
//         }

//         if (targetPose != null) {
//             PathConstraints constraints = new PathConstraints(.5, 0.5, 0.5, 0.5);
//             followLeftPath = AutoBuilder.pathfindToPose(targetPose, constraints, 0.01);
//             followLeftPath.schedule();
//         }
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (followLeftPath != null) {
//             followLeftPath.cancel();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         // End the command when the button is not pressed
//         return !driverL2.getAsBoolean();
//     }
// }
