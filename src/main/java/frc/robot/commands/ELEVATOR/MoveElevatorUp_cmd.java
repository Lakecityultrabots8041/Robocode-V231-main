package frc.robot.commands.ELEVATOR;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

/**
 * Command to move the elevator up while a button is held.
 * Uses velocity control rather than position control.
 */
public class MoveElevatorUp_cmd extends Command {
    private final Elevator_Subsystem elevator;
    private final double speed;

    /**
     * Creates a new MoveElevatorUp command.
     * 
     * @param elevator The elevator subsystem
     * @param speed The speed multiplier (0.0 to 1.0)
     */
    public MoveElevatorUp_cmd(Elevator_Subsystem elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Starting elevator up movement");
    }

    @Override
    public void execute() {
        elevator.moveUp(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotion();
        System.out.println("Elevator up movement ended");
    }

    @Override
    public boolean isFinished() {
        // Command runs until the button is released
        return false;
    }
}
