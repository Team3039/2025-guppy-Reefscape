// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Utilitys {
    public static LimelightHelpers.PoseEstimate mt2;
    public LimelightHelpers.PoseEstimate leftPose;
    public LimelightHelpers.PoseEstimate rightPose;
    public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[0];
    public SwerveDrivePoseEstimator m_poseEstimator;

    // public static Pose2d shiftPoseLeft(Pose2d originalPose, double forwardInches, double rightInches) {
    //     // Get current pose components
    //     double x = originalPose.getX();
    //     double y = originalPose.getY();
    //     Rotation2d theta = originalPose.getRotation();
    //     Rotation2d invTheta = theta.fromRadians(theta.getRadians() + Math.PI);

    //     // Compute new coordinates
    //     // Convert inches to meters (WPILib uses meters)
    //     double forwardMeters = Units.inchesToMeters(forwardInches);
    //     double rightMeters = Units.inchesToMeters(rightInches);

    //     // Calculate new position  shift target to it's right
    //     double xNew = x + forwardMeters * Math.cos(theta.getRadians()) + rightMeters * Math.sin(theta.getRadians());
    //     double yNew = y + forwardMeters * Math.sin(theta.getRadians()) - rightMeters * Math.cos(theta.getRadians());

    //     // Return the new pose with the same orientation
    //     return new Pose2d(xNew, yNew, invTheta);
    // }

    public static Pose2d shiftPoseRight(Pose2d originalPose, double forwardInches, double leftInches) {
        // Get current pose components


        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(leftInches);
        Rotation2d invTheta = theta.fromRadians(theta.getRadians() + Math.PI);

        // Compute new coordinates (shift left)  shift target to it's left
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) - rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) + rightMeters * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, invTheta);
    }
    

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Alpha",true);

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.

    // NOT quite ready to drive it

    // Command driveit = AutoBuilder.pathfindToPose(where, constraints);
   
    

    public static Pose2d getAprilTagPose(int tagID) {
        try {

            // Get the tag pose
            Optional<Pose2d> tagPose = TunerConstants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());

            return tagPose.orElse(null); // return the pose if found, otherwise null
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static double distanceToTag(int tagID) {
        Optional<Pose2d> tagPose = TunerConstants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());
        Pose2d botPose = RobotContainer.drivetrain.botPose2d;
        Translation2d targetTranslation = tagPose.get().getTranslation();
        Translation2d botTranslation = botPose.getTranslation();
        return botTranslation.getDistance(targetTranslation);
    }

    public String bestCamera(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return "limelight";
        } else if (right == null) {
            return "limelight";
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return "left";
            } else {
                return "right";
            }
        }
    }

    public PoseEstimate bestEstimate(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return right;
        } else if (right == null) {
            return left;
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return left;
            } else {
                return right;
            }
        }
    }


    public Rotation2d getGyroYaw(Pigeon2 gyro) {
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public PoseEstimate grabPose(String camera) {
        LimelightHelpers.SetRobotOrientation(camera, RobotContainer.drivetrain.gyro.getYaw().getValueAsDouble(), 0, 0,
                0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-left",getGyroYaw().getDegrees(),
        // 0, 0, 0, 0, 0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
        return mt2;

    }

}