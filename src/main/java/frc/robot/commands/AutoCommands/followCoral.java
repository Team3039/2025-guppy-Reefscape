// package frc.robot.commands.AutoCommands;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class followCoral extends Command {

//     private static final double kHeadingP =3.0;  
//     private static final double kHeadingI = 0.0;
//     private static final double kHeadingD = 0.0;
//     private static final double kForwardSpeed = 1.0; 
//     private static final double kAlignTolerance = 1.0; // degrees
//     private static final double kValidTargetThreshold = 0.5;

//     private final CommandSwerveDrivetrain drivetrain;
//     private final NetworkTableEntry tvEntry;
//     private final NetworkTableEntry txEntry;

//     private final SwerveRequest.FieldCentricFacingAngle faceTargetRequest =
//         new SwerveRequest.FieldCentricFacingAngle()
//             .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withHeadingPID(kHeadingP, kHeadingI, kHeadingD);

//     private final SwerveRequest.RobotCentricFacingAngle driveForwardRequest =
//         new SwerveRequest.RobotCentricFacingAngle()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withHeadingPID(kHeadingP, kHeadingI, kHeadingD);

//     public followCoral(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
//         addRequirements(drivetrain);

//         NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
//         this.tvEntry = limelight.getEntry("tv");
//         this.txEntry = limelight.getEntry("tx");
//     }

//     @Override
//     public void execute() {
//         double tv = tvEntry.getDouble(0.0);
//         double tx = txEntry.getDouble(0.0);


//         if (tv > kValidTargetThreshold) {

//             if (Math.abs(tx) <= kAlignTolerance) {

//                 System.out.println("kAlignTolerance");
//                 // Aligned → drive forward
//                 drivetrain.setControl(
//                     driveForwardRequest(kForwardSpeed)
//                                     );
//                                     Timer.delay(.3);
//                                 } else {
                    
//                                     System.out.println("rotateing");
                    
//                                     // Not aligned → adjust heading
//                                     Rotation2d targetHeading =
//                                         drivetrain.getPigeon2().getRotation2d().minus(Rotation2d.fromDegrees(tx));
                    
//                                     drivetrain.setControl(
//                                         faceTargetRequest.withTargetDirection(targetHeading)
//                                     );
//                                 }
//                             }
//                         }
                    
                        
                    
//                         @Override
//     public boolean isFinished() {
//         return false; 
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }
// }

















// // package frc.robot.commands.AutoCommands;

// // import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// // import com.ctre.phoenix6.swerve.SwerveRequest;
// // import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

// // import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.networktables.NetworkTable;
// // import edu.wpi.first.networktables.NetworkTableInstance;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.CommandSwerveDrivetrain;

// // public class followCoral extends Command {

// //     private final CommandSwerveDrivetrain m_drivetrain;
// //     private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

// //     private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
// //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
// //         .withDriveRequestType(DriveRequestType.Velocity)
// //         .withHeadingPID(4, 0, 0); // Tune this PID for your robot

// //     // Alignment tolerance in degrees

// //     public followCoral(CommandSwerveDrivetrain drivetrain) {
// //         this.m_drivetrain = drivetrain;
// //         addRequirements(drivetrain);



        
// //     }

// //     @Override
// //     public void initialize() {}

// //     @Override
// //     public void execute() {
// //         double tv = limelight.getEntry("tv").getDouble(0.0);

// //         double tx = limelight.getEntry("tx").getDouble(0.0);

// //         System.out.println(tx);

// //         // boolean aligned = Math.abs(tx) < 0;
// //         if (tv > 0.8) {

// //             Rotation2d targetHeading = m_drivetrain.getPigeon2().getRotation2d().minus(Rotation2d.fromDegrees(tx));
        

// //             m_drivetrain.setControl(
// //                 m_request
// //                     .withTargetDirection(targetHeading));
           

// //         }
       
        
// //         if (tx < .5 && tx > -.5 && tv > 0.8) {

// //             double forwardSpeed = 1.0; // tune this value

// //             m_drivetrain.setControl(
// //                 new SwerveRequest.RobotCentricFacingAngle()
// //                     .withDriveRequestType(DriveRequestType.Velocity)
// //                     .withHeadingPID(4, 0, 0) // tune this PID
// //                     .withVelocityX(forwardSpeed) // robot forward
// //             );
    
        
// //         }
// //         }
    

// //     @Override
// //     public boolean isFinished() {
// //         return false;
// //     }

// //     @Override
// //     public void end(boolean interrupted) {
// //     }
// // }
