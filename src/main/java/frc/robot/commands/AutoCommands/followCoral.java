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

    public followCoral(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);



        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double tv = limelight.getEntry("tv").getDouble(0.0);

        double tx = limelight.getEntry("tx").getDouble(0.0);

        System.out.println(tx);

        // boolean aligned = Math.abs(tx) < 0;
        if (tv > 0.8) {

            Rotation2d targetHeading = m_drivetrain.getPigeon2().getRotation2d().minus(Rotation2d.fromDegrees(tx));
        

            m_drivetrain.setControl(
                m_request
                    .withTargetDirection(targetHeading));
           

        }
       
        
        if (tx < .5 && tx > -.5 && tv > 0.8) {

            double forwardSpeed = 1.0; // tune this value

            // m_drivetrain.setControl(
            //     new SwerveRequest.RobotCentricFacingAngle()
            //         .withDriveRequestType(DriveRequestType.Velocity)
            //         .withHeadingPID(4, 0, 0) // tune this PID
            //         .withVelocityX(forwardSpeed) // robot forward
            // );
    
        
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
