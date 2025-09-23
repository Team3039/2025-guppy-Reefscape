package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class followCoral extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); // Tune this PID for your robot

    // Alignment tolerance in degrees
    private static final double ANGLE_TOLERANCE = 1.0;

    public followCoral(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double tx = limelight.getEntry("tx").getDouble(0.0);

        boolean aligned = Math.abs(tx) < ANGLE_TOLERANCE;

        if (!aligned) {

            m_drivetrain.setControl(
                m_request
                    .withVelocityX(0.0) 
                    .withTargetDirection(Rotation2d.fromDegrees(tx)) // turn until centered
            );
        } else {
            // When alignen drive forward
            m_drivetrain.setControl(
                m_request
                    .withVelocityX(1.0) 
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
