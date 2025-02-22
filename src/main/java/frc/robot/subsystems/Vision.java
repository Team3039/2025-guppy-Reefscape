package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

    public static final Pose3d FrontReefBlue = new Pose3d(7.043 , 7.358 , 2.12, new Rotation3d());
    public static final Pose3d FrontReefRed = new Pose3d(16.3, 5.549, 2.12, new Rotation3d());
    static Pose3d desiredReefPose;

    public enum VisionState {
        DRIVING,
        ROTATING
    }

    VisionState visionState = VisionState.DRIVING;

    
    static double distance = 0;
    static double targetYaw;
    public static double yawOffset = -1 * Units.degreesToRadians(11);

    public static double rotation = 0;
    public static boolean shouldRotateToReef = false;
    int indexID;
    double desiredAllianceID;

    public static PIDController targetAlignment = new PIDController(10, 0, 0.00);

    /** Creates a new Vision. */
    public Vision() {
        setState(VisionState.DRIVING);
    }

    public VisionState getState() {
        return visionState;
    }

    public void setState(VisionState state) {
        visionState = state;
    }

    public static Translation3d getLimelightPose() {
        double[] defaultValue = new double[6];
        double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultValue);
        if (pose.length == 6) {
            return new Translation3d(pose[0], pose[1], pose[2]);
        }
        return null;
    }

    public boolean isAtRotationSetpoint() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            desiredReefPose = FrontReefBlue;
        } else {
            desiredReefPose = FrontReefRed;
        }

        return (Math.abs(Math.atan((RobotContainer.drivetrain.getState().Pose.getY() - desiredReefPose.getY())
                / (RobotContainer.drivetrain.getState().Pose.getX() - desiredReefPose.getX())) -
                RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())) < .5;
    }

    public static double getDistanceToSpeaker() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            desiredReefPose = FrontReefBlue;
        } else {
            desiredReefPose = FrontReefRed;
        }

        double SpeakerX = desiredReefPose.getX();
        double distanceXToSpeaker = SpeakerX - RobotContainer.drivetrain.getState().Pose.getX();
        double SpeakerY = desiredReefPose.getY();
        double distanceYToSpeaker = SpeakerY - RobotContainer.drivetrain.getState().Pose.getY();

        distance = Math.hypot(distanceXToSpeaker, distanceYToSpeaker);
        return distance;
    }

    public static double getRotationToSpeaker() {
        if (shouldRotateToReef || RobotContainer.driverPad.rightBumper().getAsBoolean()) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                desiredReefPose = FrontReefBlue;
            } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                desiredReefPose = FrontReefRed;
            }

            targetYaw = Math.atan((RobotContainer.drivetrain.getState().Pose.getY() -
                    desiredReefPose.getY()) /
                    (RobotContainer.drivetrain.getState().Pose.getX() - desiredReefPose.getX()));

            rotation = 1.0 * targetAlignment
                    .calculate(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians(), targetYaw);
        } else {
            rotation = -RobotContainer.driverPad.getRightX() * TunerConstants.MaxAngularRate;
        }
        return rotation;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Current Robot Pose",
                RobotContainer.drivetrain.getState().Pose.toString());
        SmartDashboard.putBoolean("Is At Rotation Setpoint", isAtRotationSetpoint());

        switch (visionState) {
            case DRIVING:
            desiredReefPose = null;
                break;
            case ROTATING:
                getRotationToSpeaker();
                desiredReefPose = null;
                break;
        }
    }
}







































// man I love pies how about you 