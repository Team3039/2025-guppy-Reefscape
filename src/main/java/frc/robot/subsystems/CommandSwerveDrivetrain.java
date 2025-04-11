package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import java.io.IOException;
import java.util.function.Supplier;

// import org.json.simple.parser.ParseException;
// import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
// @SuppressWarnings("unused")
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

        public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];

    public PoseEstimate cameraPose;

    /* Blue alliance sees forward as 180 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Red alliance sees forward as 0 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // Declare currentPose as a Pose2d object

    // /* Swerve requests to apply during SysId characterization */
    // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         // state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    //@SuppressWarnings("unused")
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    //@SuppressWarnings("unused")
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             // SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    /* The SysId routine to test */
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
        public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */

    // //If(1==1){ the world is flat }


    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    

    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * 
     * 
     * 
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(2, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(2, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements

                

            );


            

        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }


    
    // // Define the cameraPoses array and other required objects
    // private CameraPose[] cameraPoses = new CameraPose[2];
    // private Gyro gyro = new Gyro(); // Replace with your actual Gyro class
    // private PoseEstimator m_poseEstimator = new PoseEstimator(); // Replace with your actual PoseEstimator class

    // // Define the grabPose method
    // private CameraPose grabPose(String cameraName) {
    //     // Replace with your actual implementation for grabbing a camera pose
    //     return new CameraPose();
    // }

    // public void updateCameraPose() {
    //     boolean doRejectUpdate = false;
    //     int bestCamera;
    //     double leftAmbiguity = 0;
    //     double rightAmbiguity = 0;
    //     cameraPoses[0] = grabPose("limelight");
    // //    cameraPoses[1] = grabPose("limelight-right");

    // //     if (cameraPoses[0] == null && cameraPoses[1] == null) {
    // //         bestCamera = -1;
    // //     } else if (cameraPoses[0] == null) {
    // //         bestCamera = 1;
    // //     } else if (cameraPoses[1] == null) {
    // //         bestCamera = 0;
    // //     } else {
    // //         if (cameraPoses[0].tagCount > 0) {
    // //             leftAmbiguity = cameraPoses[0].rawFiducials[0].ambiguity;
    // //         }
    // //         if (cameraPoses[1].tagCount > 0) {
    // //             rightAmbiguity = cameraPoses[1].rawFiducials[0].ambiguity;
    // //         }
    // //         if (leftAmbiguity < rightAmbiguity) {
    // //             bestCamera = 0;
    // //         } else {
    // //             bestCamera = 1;
    // //         }
    // //     }
    // //     if (bestCamera == -1) {
    // //         doRejectUpdate = true;
    // //     }else {
    // //         if(cameraPoses[bestCamera].tagCount<1){
    // //             doRejectUpdate=true;
    // //         }
    // //     }
        
    // //     // for (int i = 0; i < 2; i++) {

    //     // doRejectUpdate = false;
    //     // if (cameraPoses[i] != null) {
    //     // if (cameraPoses[i].tagCount == 0) {
    //     // doRejectUpdate = true;
    //     // }
    //     // if (cameraPoses[i].pose.getX() < 0) {
    //     // doRejectUpdate = true;
    //     // }
    //     // if (cameraPoses[i].pose.getY() > 7.6) {
    //     // doRejectUpdate = true;
    //     // }
    //     // } else {
    //     // doRejectUpdate = true;
    //     // }
    //     /// SmartDashboard.putNumber("estimated yaw",
    //     // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    //     if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our angular velocity is greater
    //     {
    //         doRejectUpdate = true;
    //     }
    //     SmartDashboard.putBoolean("RejectUpdate", doRejectUpdate);
    //     if (!doRejectUpdate) {
    //         SmartDashboard.putNumber("bestcamera",bestCamera);
    //         SmartDashboard.putNumberArray("CameraPose", new double[] { cameraPoses[bestCamera].pose.getTranslation().getX(), cameraPoses[bestCamera].pose.getTranslation().getY(),
    //             cameraPoses[bestCamera].pose.getRotation().getRadians() });
    //         m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999));
    //         m_poseEstimator.addVisionMeasurement(
    //                 cameraPoses[bestCamera].pose,
    //                 cameraPoses[bestCamera].timestampSeconds);
    //         // resetOdometry(m_poseEstimator.getEstimatedPosition());
    //         // swerveOdometry.resetPosition(getGyroRotation2D(), getModulePositions(),
    //         // m_poseEstimator.getEstimatedPosition());
    //     }

    // }




    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        // return m_sysIdRoutineToApply.quasistatic(direction);
    // }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }

    @Override
    public void periodic() {
        


        Pose2d currentPose = getState().Pose; // Get the current pose from the drivetrain state
        SmartDashboard.putNumber("Swerve Odometry X", currentPose.getX());
        SmartDashboard.putNumber("Swerve Odometry Y", currentPose.getY());
        SmartDashboard.putNumber("Swerve Odometry Rotation (Degrees)", currentPose.getRotation().getDegrees());


        /*

         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Blue
                        ? kBlueAlliancePerspectiveRotation
                        : kRedAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
    //  */
    @Override
    public void addVisionMeasurement(Pose2d currentPose, double timestampSeconds) {
        super.addVisionMeasurement(currentPose, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d currentPose,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(currentPose, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}