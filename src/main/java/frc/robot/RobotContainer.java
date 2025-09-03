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
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetClawIntakeAlgae;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.commands.SetClawRelease;
import frc.robot.commands.SetClimbManualOverride;
import frc.robot.commands.AutoCommands.CoralintakeAuto;
import frc.robot.commands.AutoCommands.LeftBranchPathFinding;
// import frc.robot.commands.AutoCommands.PathFindToProcessor;
import frc.robot.commands.AutoCommands.RightBranchPathfinding;
// import frc.robot.commands.PathFinding.rightBranchPathfinding;
// import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL2;
// import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL2;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL4;
import frc.robot.commands.ElevatorRoutines.ScoreCoralTrough;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralClaw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;


public class RobotContainer {
  

// private static final Logger logger = Logger.getLogger(RobotContainer.class.getName());

private final SendableChooser<Command> autoChooser;


APTarget ApReef = new APTarget(TunerConstants.POSES.REEF_F)
  .withEntryAngle(Rotation2d.kZero);


public RobotContainer() {


    // reset pose to start 

    NamedCommands.registerCommand("resetGyroAPtesting", drivetrain.new resetGyroAPtesting());

    NamedCommands.registerCommand("R-P mid pipething", drivetrain.new ResetOdometry(drivetrain, new Pose2d(7.569, 0.517, Rotation2d.fromDegrees(0))));

    NamedCommands.registerCommand("Mid pipething set gyro", (drivetrain.runOnce(() -> drivetrain.resetGyroMidPipeThing())));
    

    // auto pathing commands

    NamedCommands.registerCommand("LeftBranch Pathfinding", new LeftBranchPathFinding(drivetrain));
    NamedCommands.registerCommand("RightBranch Pathfinding", new RightBranchPathfinding(drivetrain));

//
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
    SmartDashboard.putData("Auto Chooser", autoChooser);

        
    configureBindings();
    
}
    
public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); 

    public final static CommandXboxController operatorPad = new CommandXboxController(1);

    public final static CommandXboxController PitPad = new CommandXboxController(2);



  /* Operator Buttons */
  private final JoystickButton driverX = new JoystickButton(driverPad, PS4Controller.Button.kCross.value);
//   private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kSquare.value);
//   private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Controller.Button.kTriangle.value);
  private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Controller.Button.kCircle.value);

  public static final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Controller.Button.kL1.value);
  public static final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Controller.Button.kR1.value);

  public final static JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
  private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kR2.value);
//   private final JoystickButton driverR3 = new JoystickButton(driverPad, PS4Controller.Button.kR3.value);

//   private final JoystickButton driverPadButton = new JoystickButton(driverPad,  PS4Controller.Button.kTouchpad.value);
//   private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Controller.Button.kPS.value);

  public final static JoystickButton driverShare = new JoystickButton(driverPad, PS4Controller.Button.kShare.value);
  private final JoystickButton driverOptions = new JoystickButton(driverPad, PS4Controller.Button.kOptions.value);





    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);





    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Wrist wrist = new Wrist();
    public static final CoralClaw claw = new CoralClaw();
    public static final Climb climb = new Climb();
    // public static final Limelight limelight = new Limelight();


    /* Path follower */
    // private final SendableChooser<Command> autoChooser;



   

    private void configureBindings() {






    
        // Drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

//Driver pad 


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverPad.interpolatedLeftYAxis() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverPad.interpolatedLeftXAxis() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(driverPad.interpolatedRightXAxis() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

      
        


      

        driverR2.whileTrue (new RightBranchPathfinding(drivetrain));


        driverL2.whileTrue (new LeftBranchPathFinding(drivetrain));

        


        // driverSquare.onTrue (new PathFindToProcessor()); 

        driverCircle.onTrue (drivetrain.new ResetOdometry(drivetrain, new Pose2d(3.192, 4.005, Rotation2d.fromDegrees(0))));

        driverX.whileTrue(new SetClawIntakeAlgae());

        // driverTriangle.whileTrue(new SetClawRelease());

    
        driverOptions.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        driverShare.toggleOnTrue(new SetClimbManualOverride());

        // drivetrain.registerTelemetry(logger::telemeterize); // Commented out as Logger does not have telemeterize method








        //pit pad


        PitPad.y().toggleOnTrue(new SetClimbManualOverride());
        




    //My controls 

        // Overrides
        // operatorPad.rightStick().toggleOnTrue(new SetElevatorManualOverride());
        // operatorPad.leftStick().toggleOnTrue(new SetWristManualOverride());
        
        // intake and release
        operatorPad.b().whileTrue(new SetClawIntakeCoral());
        operatorPad.a().whileTrue(new SetClawRelease());
        // operatorPad.leftTrigger().whileTrue(new SetClawIntakeAlgae());



        //clear algae
        // operatorPad.x().whileTrue(new RemoveAlgaeL2());
        // operatorPad.y().whileTrue(new RemoveAlgaeL3());
        // operatorPad.rightBumper().onTrue(new ScoreAlgaeBarge());

        

        // Scoring Coral
        operatorPad.povDown().onTrue(new ScoreCoralTrough());
        operatorPad.povLeft().onTrue(new ScoreCoralL2());
        operatorPad.povRight().onTrue(new ScoreCoralL3());
        operatorPad.povUp().onTrue(new ScoreCoralL4());



        
    // //My controls 

    //     // // Overrides
    //     // operatorPad.rightStick().toggleOnTrue(new SetElevatorManualOverride());
    //     // operatorPad.leftStick().toggleOnTrue(new SetWristManualOverride());
        
    //     // intake and release
    //     // operatorPad.a().whileTrue(new SetClawIntakeCoral());
    //     // operatorPad.b().whileTrue(new SetClawRelease());
    //     // operatorPad.leftTrigger().whileTrue(new SetClawIntakeAlgae());

    //     operatorPad.povUp().and(operatorPad.leftBumper()).whileTrue(new SetClawIntakeCoral());
    //     operatorPad.povDown().and(operatorPad.leftBumper()).whileTrue(new SetClawRelease());


    //     //clear algae
    //     // operatorPad.x().whileTrue(new RemoveAlgaeL2());
    //     // operatorPad.y().whileTrue(new RemoveAlgaeL3());

    //     operatorPad.povUp().and(operatorPad.b()).onTrue(new RemoveAlgaeL2());
    //     operatorPad.povUp().and(operatorPad.a()).onTrue(new RemoveAlgaeL3());

    //     // operatorPad.rightBumper().onTrue(new ScoreAlgaeBarge());

    //     //ScoreCoralTrough

    //     // Scoring Coral
    //     // operatorPad.povDown().and(operatorPad.x()).onTrue(new ScoreCoralTrough());
    //     operatorPad.povDown().and(operatorPad.y()).onTrue(new ScoreCoralTrough());
    //     operatorPad.povDown().and(operatorPad.b()).onTrue(new ScoreCoralTrough());
    //     operatorPad.povDown().and(operatorPad.a()).onTrue(new ScoreCoralTrough());

    //     // operatorPad.povDown().and(operatorPad.x()).onTrue(new ScoreCoralTrough());
    //     operatorPad.povUp().and(operatorPad.y()).onTrue(new ScoreCoralL2());
    //     operatorPad.povUp().and(operatorPad.b()).onTrue(new ScoreCoralL3());
    //     operatorPad.povUp().and(operatorPad.a()).onTrue(new ScoreCoralL4());

    //     operatorPad.povUp().and(operatorPad.x()).onTrue(new RemoveAlgaeL3());
    //     operatorPad.povDown().and(operatorPad.x()).onTrue(new RemoveAlgaeL2());        

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}