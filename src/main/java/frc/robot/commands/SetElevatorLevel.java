package frc.robot.commands;

import frc.robot.subsystems.elevator.elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorLevel extends Command {

    private final elevator elevator;
    private final double targetHeight;
    private final double tolerance = 0.05; // in meters

    public SetElevatorLevel(elevator elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetHeight(targetHeight);
    }

    @Override
    public void execute() {
        // Optionally log progress or monitor the elevator's motion here.
    }

    @Override
    public boolean isFinished() {
        // Command is finished once the elevator is within tolerance of the target.
        return Math.abs(elevator.getCurrentHeight() - targetHeight) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally, you can stop or hold the elevator here.
    }
}