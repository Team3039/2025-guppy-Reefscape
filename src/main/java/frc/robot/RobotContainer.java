// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
// import edu.wpi.first.units.Units;




import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.util.FileVersionException;

// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetClawIntakeAlgae;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.commands.SetClawRelease;
// import frc.robot.commands.SetClawSpitAlgae;
import frc.robot.commands.SetClimbManualOverride;
// import frc.robot.commands.SetElevatorManualOverride;
// import frc.robot.commands.SetWristManualOverride;
// import frc.robot.commands.StopPathFinding;
// import frc.robot.commands.PathFinding.rightBranchPathfinding;
import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL2;
import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL3;
// import frc.robot.commands.ElevatorRoutines.ScoreAlgaeBarge;
// import frc.robot.commands.ElevatorRoutines.ScoreAlgaeProcessor;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL2;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL4;
import frc.robot.commands.ElevatorRoutines.ScoreCoralTrough;
// import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.CommandSwerveDrivetrain.ResetOdometry;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AutoCommands.*;
import frc.robot.subsystems.Wrist;


public class RobotContainer {
  

// private static final Logger logger = Logger.getLogger(RobotContainer.class.getName());

private final SendableChooser<Command> autoChooser;



public RobotContainer() {


    // reset pose to start 

    NamedCommands.registerCommand("R-P mid pipething", drivetrain.new ResetOdometry(drivetrain, new Pose2d(7.589, 1.169, Rotation2d.fromDegrees(180))));


    //Elevator commands
    NamedCommands.registerCommand("score L2", new ScoreCoralL2());
    NamedCommands.registerCommand("score L3", new ScoreCoralL3());
    NamedCommands.registerCommand("score L4", new ScoreCoralL4());
    NamedCommands.registerCommand("set Wrist/Elevator down", new ScoreCoralTrough());


    //itake and spit coral             (intake)
    NamedCommands.registerCommand("LoveCrushingLoaf", new CoralintakeAuto());
    NamedCommands.registerCommand("WaterBucketRelease", new SetClawRelease());
    //                                  (Release)


     autoChooser = AutoBuilder.buildAutoChooser(); //Auto chooser
    SmartDashboard.putData("Auto Mode", autoChooser);

        
    configureBindings();
    
}
    
public static final CommandXboxController driverPad = new CommandXboxController(0); 

    public final static CommandXboxController operatorPad = new CommandXboxController(1);

    public final static CommandXboxController PitPad = new CommandXboxController(2);



  /* Operator Buttons */
  




    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()


            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);





    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Wrist wrist = new Wrist();
    public static final Claw claw = new Claw();
    public static final Climb climb = new Climb();
    // public static final Limelight limelight = new Limelight();


    /* Path follower */
    // private final SendableChooser<Command> autoChooser;



public void periodic() {

    drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).schedule(); // Ensure field-centric is seeded at the start of each periodic call

}
   

    private void configureBindings() {






    
        // Drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

//Driver pad 


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverPad.getRightTriggerAxis() * MaxSpeed) // Drive forward with negative Y (forward)

                     .withVelocityX(driverPad.getLeftTriggerAxis() * MaxSpeed) // Drive forward with positive Y (forward)

                     .withRotationalRate(-driverPad.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //     drive.withVelocityX(-operatorPad.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //         .withVelocityY(-operatorPad.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //         .withRotationalRate(operatorPad.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );


      

        driverPad.rightBumper().whileTrue (new RightBranchPathfinding());


        driverPad.leftBumper().whileTrue (new LeftBranchPathFinding());


        // driverSquare.onTrue (new PathFindToProcessor()); 

        // driverCircle.onTrue (drivetrain.new ResetOdometry(drivetrain, new Pose2d(3.192, 4.005, Rotation2d.fromDegrees(0))));

        // driverSquare.whileTrue(new SetClawIntakeCoral());
        // driverTriangle.whileTrue(new SetClawRelease());

    
        // driverOptions.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // driverShare.toggleOnTrue(new SetClimbManualOverride());

        // drivetrain.registerTelemetry(logger::telemeterize); // Commented out as Logger does not have telemeterize method








        //pit pad


        PitPad.y().toggleOnTrue(new SetClimbManualOverride());
        




    //My controls 

        // // Overrides
        // operatorPad.rightStick().toggleOnTrue(new SetElevatorManualOverride());
        // operatorPad.leftStick().toggleOnTrue(new SetWristManualOverride());
        
        // intake and release
        operatorPad.a().whileTrue(new SetClawIntakeCoral());
        operatorPad.b().whileTrue(new SetClawRelease());
        operatorPad.leftTrigger().whileTrue(new SetClawIntakeAlgae());



        //clear algae
        operatorPad.x().whileTrue(new RemoveAlgaeL2());
        operatorPad.y().whileTrue(new RemoveAlgaeL3());
        // operatorPad.rightBumper().onTrue(new ScoreAlgaeBarge());

        

        // Scoring Coral
        operatorPad.povDown().onTrue(new ScoreCoralTrough());
        operatorPad.povLeft().onTrue(new ScoreCoralL2());
        operatorPad.povRight().onTrue(new ScoreCoralL3());
        operatorPad.povUp().onTrue(new ScoreCoralL4());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}