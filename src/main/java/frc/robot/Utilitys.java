package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.WonderOnOverToConstants;

public class Utilitys {
    public static LimelightHelpers.PoseEstimate mt2;
    public LimelightHelpers.PoseEstimate leftPose;
    public SwerveDrivePoseEstimator m_poseEstimator;

    public static Pose2d shiftPoseLimelight(Pose2d originalPose, double forwardInches, double rightInches) {
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation();
        Rotation2d invTheta = Rotation2d.fromRadians(theta.getRadians() + Math.PI);

        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(rightInches);

        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) + rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) - rightMeters * Math.cos(theta.getRadians());

        return new Pose2d(xNew, yNew, invTheta);
    }

    public static Command driveToIt(boolean right) {
        PathConstraints constraints = new PathConstraints(
            1.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        double leftDist;
        boolean validTarget = false;
        int tagId = -1;
        Pose2d where = RobotContainer.drivetrain.getPose();

        LimelightHelpers.LimelightResults resultsLeft = LimelightHelpers.getLatestResults("limelight-left");

        if (resultsLeft.valid && resultsLeft.targets_Fiducials.length > 0) {
            leftDist = resultsLeft.botpose_avgdist;
            tagId = (int) resultsLeft.targets_Fiducials[0].fiducialID;
            validTarget = true;
        } else {
            leftDist = 999999;
        }

        if (validTarget) {
            double offset = right ? WonderOnOverToConstants.rightOffset : WonderOnOverToConstants.leftOffset;
            where = Utilitys.shiftPoseLimelight(Utilitys.getAprilTagPose(tagId), WonderOnOverToConstants.forwardOffset, offset);

            SmartDashboard.putNumberArray("Where", new double[]{where.getX(), where.getY(), where.getRotation().getRadians()});
            return AutoBuilder.pathfindToPose(where, constraints, 0.0);
        }

        return null;
    }

    public static Pose2d getAprilTagPose(int tagID) {
        try {
            Optional<Pose2d> tagPose = TunerConstants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());
            return tagPose.orElse(null);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static double distanceToTag(int tagID) {
        Optional<Pose2d> tagPose = TunerConstants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());
        Pose2d botPose = RobotContainer.drivetrain.botPose2d;
        return botPose.getTranslation().getDistance(tagPose.get().getTranslation());
    }

    public static int grabTagID() {
        double leftDist;
        int tagId = -1;
        boolean validTarget = false;

        LimelightHelpers.LimelightResults resultsLeft = LimelightHelpers.getLatestResults("limelight-left");

        if (resultsLeft.valid && resultsLeft.targets_Fiducials.length > 0) {
            leftDist = resultsLeft.botpose_avgdist;
            tagId = (int) resultsLeft.targets_Fiducials[0].fiducialID;
            validTarget = true;
            SmartDashboard.putString("Camera", "left");
        } else {
            leftDist = 999999;
        }

        SmartDashboard.putBoolean("valid", validTarget);
        return tagId;
    }

    public Rotation2d getGyroYaw(Pigeon2 gyro) {
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public PoseEstimate grabPose(String camera) {
        LimelightHelpers.SetRobotOrientation(camera, RobotContainer.drivetrain.gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
        return mt2;
    }
}
