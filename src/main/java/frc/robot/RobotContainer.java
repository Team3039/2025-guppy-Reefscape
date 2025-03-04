// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.SetClawIntakeAlgae;
import frc.robot.commands.SetClawIntakeCoral;
import frc.robot.commands.SetElevatorManualOverride;
import frc.robot.commands.SetWristManualOverride;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL2;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL3;
import frc.robot.commands.ElevatorRoutines.ScoreCoralL4;
import frc.robot.commands.ElevatorRoutines.ScoreCoralTrough;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
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

    public final static CommandXboxController driverPad = new CommandXboxController(0);
    public final static CommandXboxController operatorPad = new CommandXboxController(1);


    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Wrist wrist = new Wrist();
    public static final Claw claw = new Claw();

// MAKE THE ELAVATOR WIRES 12FT LONG (Young told someone to write it down)


    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");



        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverPad.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverPad.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverPad.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverPad.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverPad.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverPad.getLeftY(), -driverPad.getLeftX()))
        ));

        driverPad.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverPad.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverPad.back().and(driverPad.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverPad.back().and(driverPad.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverPad.start().and(driverPad.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverPad.start().and(driverPad.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverPad.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Overrides
        operatorPad.povLeft().toggleOnTrue(new SetElevatorManualOverride());
        operatorPad.povRight().toggleOnTrue(new SetWristManualOverride());

        // Intaking
        operatorPad.leftTrigger().whileTrue(new SetClawIntakeCoral());
        operatorPad.leftBumper().whileTrue(new SetClawIntakeAlgae());

        // Scoring Coral
        // operatorPad.povDown().onTrue(new ScoreCoralTrough());
        // operatorPad.povRight().onTrue(new ScoreCoralL2());
        // operatorPad.povLeft().onTrue(new ScoreCoralL3());
        // operatorPad.povUp().onTrue(new ScoreCoralL4());
    }

     public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return null;
    }
}
