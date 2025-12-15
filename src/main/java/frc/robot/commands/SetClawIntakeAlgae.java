// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Claw.ClawState;
// import frc.robot.subsystems.Wrist.WristState;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class SetClawIntakeAlgae extends Command {

//     public final static CommandXboxController operatorPad = new CommandXboxController(1);


//   public SetClawIntakeAlgae() {
//     addRequirements(RobotContainer.claw);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (!RobotContainer.claw.hasGamepiece()) {
//       RobotContainer.claw.setState(ClawState.ALGAE);
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     if (!RobotContainer.claw.hasGamepiece()) {

//       RobotContainer.claw.setState(ClawState.PASSIVE);
//       RobotContainer.wrist.setState(WristState.PASSIVE);

      
//   }
//     else {
//       RobotContainer.claw.setState(ClawState.PASSIVE);
//       RobotContainer.wrist.setState(WristState.PASSIVE);
//     }

//     if (RobotContainer.operatorPad.rightTrigger().getAsBoolean());

//   }



//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
