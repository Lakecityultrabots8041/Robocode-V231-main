package frc.robot.commands;

import frc.robot.subsystems.elevator.elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class AdjustElevator extends Command {
    private final elevator elevator;
    private final double delta; // Delta in meters (or appropriate units)

    public AdjustElevator(elevator elevator, double delta) {
        this.elevator = elevator;
        this.delta = delta;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Read the current target. Depending on your design, you might store the
        // current target in your elevator subsystem.
        double currentTarget = elevator.getCurrentHeight();  // or get a stored target
        double newTarget = currentTarget + delta;
        System.out.println("Adjusting elevator target by " + delta + " m to " + newTarget);
        elevator.setTargetHeight(newTarget);
    }

    @Override
    public void execute() {
        // For a one-time adjustment, you don't need to continuously run code.
    }

    @Override
    public boolean isFinished() {
        // If you want a one-shot adjustment, finish immediately.
        // If you want continuous adjustment while holding the trigger, return false.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally do cleanup if needed.
    }
}