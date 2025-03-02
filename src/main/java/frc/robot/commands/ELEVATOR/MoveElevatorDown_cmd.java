package frc.robot.commands.ELEVATOR;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

/**
 * Command to move the elevator up while a button is held.
 * Uses velocity control rather than position control.
 */
public class MoveElevatorDown_cmd extends Command {
    private final Elevator_Subsystem elevator;
    private final double speed;

    
    public MoveElevatorDown_cmd(Elevator_Subsystem elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Starting elevator down movement");
    }

    @Override
    public void execute() {
        elevator.moveDown(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotion();
        System.out.println("Elevator down movement ended");
    }

    @Override
    public boolean isFinished() {
        // Command runs until the button is released
        return false;
    }
}