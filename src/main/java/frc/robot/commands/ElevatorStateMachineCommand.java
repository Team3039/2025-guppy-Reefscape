package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorStateMachineCommand extends Command {
    private final Elevator elevator;
    
    public ElevatorStateMachineCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
        // we could set this to a default position
    }
    
    @Override
    public void execute() {
        // Elevator State Machine
        switch (elevator.getState()) {
            // In the Idle state, the elevator rests at the bottom of the robot
            case IDLE:
                Elevator.setSetpoint(0);
                elevator.setElevatorPosition();
                break;
                
            // In the Manual state, the elevator is controlled directly by the operator
            case MANUAL:
                elevator.setElevatorPercent(RobotContainer.operatorPad.getLeftY() * 0.3);
                break;
                
            // in the Position state, the elevator is controlled by the setpoint
            case POSITION:
                elevator.setElevatorPosition();
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // clean up if needed
    }
    
    @Override
    public boolean isFinished() {
        return false; // command runs until interrupted
    }
}