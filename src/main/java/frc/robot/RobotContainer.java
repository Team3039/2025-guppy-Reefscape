// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import java.util.logging.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.commands.SetClawRelease;
import frc.robot.commands.SetClimbManualOverride;
import frc.robot.commands.SetElevatorManualOverride;
import frc.robot.commands.SetWristManualOverride;
import frc.robot.commands.PathFinding.leftBranchAlign;
import frc.robot.commands.PathFinding.rightBranchAlign;
import frc.robot.commands.AutoCommands.CoralintakeAuto;
import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL2;
import frc.robot.commands.ElevatorRoutines.RemoveAlgaeL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL2;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL4;
import frc.robot.commands.ElevatorRoutines.ScoreCoralTrough;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;





public class RobotContainer {
  

private static final Logger logger = Logger.getLogger(RobotContainer.class.getName());
private final SendableChooser<Command> autoChooser;

public RobotContainer() {
  


    //Elevator commands
    NamedCommands.registerCommand("score L2", new ScoreCoralL2());
    NamedCommands.registerCommand("score L3", new ScoreCoralL3());
    NamedCommands.registerCommand("score L4", new ScoreCoralL4());
    NamedCommands.registerCommand("set Wrist/Elevator down", new ScoreCoralTrough());

    //itake and spit coral             (intake)
    NamedCommands.registerCommand("hwak", new CoralintakeAuto());
    NamedCommands.registerCommand("Tuha", new SetClawRelease());
    //                                  (Release)

    autoChooser = AutoBuilder.buildAutoChooser(); //Auto chooser
    SmartDashboard.putData("Auto Mode", autoChooser);


    //Auto paths                                
    SmartDashboard.putData("Test (Ps 1c)", new PathPlannerAuto("Test (Ps 1c)"));
//                                ^ Name for path in smartdashboard             ^ Name of path in pathplanner
configureBindings();


}
    
    
    public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); // Co-Pilot Joystick

    public final static CommandXboxController operatorPad = new CommandXboxController(1);

    public final static CommandXboxController PitPad = new CommandXboxController(2);



  /* Operator Buttons */
  private final JoystickButton driverX = new JoystickButton(driverPad, PS4Controller.Button.kCross.value);
  private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kSquare.value);
  private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Controller.Button.kTriangle.value);
  private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Controller.Button.kCircle.value);

  public static final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Controller.Button.kL1.value);
  public static final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Controller.Button.kR1.value);

  private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
  private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kR2.value);
  private final JoystickButton driverR3 = new JoystickButton(driverPad, PS4Controller.Button.kR3.value);

  private final JoystickButton driverPadButton = new JoystickButton(driverPad,
      PS4Controller.Button.kTouchpad.value);
  private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Controller.Button.kPS.value);

  private final JoystickButton driverShare = new JoystickButton(driverPad, PS4Controller.Button.kShare.value);
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
    public static final Claw claw = new Claw();
    public static final Climb climb = new Climb();
    public static final Limelight limelight = new Limelight(drivetrain);


    /* Path follower */
    // private final SendableChooser<Command> autoChooser;


    
    private void configureBindings() {
        // Drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Blue A");

            PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Math.toRadians(360), Math.toRadians(540));
               
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints);

        } catch (FileVersionException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }

       






//Driver pad 


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverPad.interpolatedLeftYAxis() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverPad.interpolatedLeftXAxis() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(driverPad.interpolatedRightXAxis() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        driverL2.whileTrue(new leftBranchAlign());
        driverR2.whileTrue(new rightBranchAlign());


        driverX.whileTrue(drivetrain.applyRequest(() -> brake));
    
        driverOptions.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize); // Commented out as Logger does not have telemeterize method

        driverShare.toggleOnTrue(new SetClimbManualOverride());


        //pit pad


        PitPad.y().toggleOnTrue(new SetClimbManualOverride());




    //My controls 

        // // Overrides
        operatorPad.rightStick().toggleOnTrue(new SetElevatorManualOverride());
        operatorPad.leftStick().toggleOnTrue(new SetWristManualOverride());
        
        // intake and release
        operatorPad.a().whileTrue(new SetClawIntakeCoral());
        operatorPad.b().whileTrue(new SetClawRelease());

        //clear algae
        operatorPad.x().whileTrue(new RemoveAlgaeL2());
        operatorPad.y().whileTrue(new RemoveAlgaeL3());

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



