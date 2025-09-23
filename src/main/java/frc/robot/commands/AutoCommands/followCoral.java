package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class followCoral extends Command {

    public void Stop(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
                addRequirements(m_drivetrain);
            
    }
            private CommandSwerveDrivetrain m_drivetrain;
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); // Tune this PID for your bot!

    public APTarget targetPose;
    public APResult apOut;

    public followCoral() {
        this.m_drivetrain = m_drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = null;
        apOut = null;
    }

    @Override
    public void execute() {
        // Grab Limelight vision data

        double tx = limelight.getEntry("tx").getDouble(0.0); // horizontal offset (deg)
        double ty = limelight.getEntry("ty").getDouble(0.0); // vertical offset (deg or distance depending on LL mode)
        double ta = limelight.getEntry("ta").getDouble(0.0);

       Pose2d coralPose = new Pose2d( tx, ty, Rotation2d.fromDegrees(tx));

        // Create an APTarget from the Pose2d
        targetPose = new APTarget(coralPose);




        if (targetPose != null) {
            // Apply autopilot outputs to the drivetrain
            m_drivetrain.setControl(
                m_request
                    .withVelocityX(tx)   // forward/backward
                    .withVelocityY(ty)   // strafe
                    .withTargetDirection(Rotation2d.fromDegrees(tx)) // desired heading
            );
        }
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {return true;}
        else {return false;}

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        Stop(m_drivetrain);
    }
}
