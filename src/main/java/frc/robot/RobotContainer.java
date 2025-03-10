// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ActuateElevatorToSetpoint;
import frc.robot.commands.SetClawIntakeAlgae;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.commands.SetClawRelease;
import frc.robot.commands.SetClimbManualOverride;
import frc.robot.commands.SetElevatorManualOverride;
import frc.robot.commands.SetWristManualOverride;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL2;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL4;
import frc.robot.commands.ElevatorRoutines.ScoreCoralTrough;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;


public class RobotContainer {
  


private final SendableChooser<Command> autoChooser;

public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser(); 
    SmartDashboard.putData("Auto Mode", autoChooser);

    NamedCommands.registerCommand("score L3", new ScoreCoralL3());
    NamedCommands.registerCommand("Tuha", new SetClawRelease());
    NamedCommands.registerCommand("set Wrist/Elevator down ", new SetClawRelease());

    SmartDashboard.putData("Test (Ps 1c)", new PathPlannerAuto("Test (Ps 1c)"));

    configureBindings();
}
    
    
    public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); // Co-Pilot Joystick

    public final static CommandXboxController operatorPad = new CommandXboxController(1);


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

    private final Telemetry logger = new Telemetry(MaxSpeed);




    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Wrist wrist = new Wrist();
    public static final Claw claw = new Claw();
    public static final Climb climb = new Climb();


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
                drive.withVelocityX(-driverPad.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverPad.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverPad.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        
        driverX.whileTrue(drivetrain.applyRequest(() -> brake));
        

        driverOptions.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);



        driverR2.whileTrue(new SetClawRelease());

        driverTriangle.toggleOnTrue(new SetClimbManualOverride());







    //My controls 

        // // Overrides
        operatorPad.rightStick().toggleOnTrue(new SetElevatorManualOverride());
        operatorPad.leftStick().toggleOnTrue(new SetWristManualOverride());

        operatorPad.a().whileTrue(new SetClawIntakeCoral());
        operatorPad.b().whileTrue(new SetClawRelease());
        
        operatorPad.y().toggleOnTrue(new SetClimbManualOverride());


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



