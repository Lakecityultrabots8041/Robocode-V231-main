package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.elevator;

public class MoveElevator extends Command{

    private final elevator elevator;
    private final double targetPosition;
    private boolean isFinished = false;

    public MoveElevator(elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator moving to target: " + targetPosition);
        elevator.setTargetHeight(targetPosition);
    }

    @Override
    public void execute() {
        // Motion Magic will handle internal control, so this is empty for now
        if (Math.abs(elevator.getCurrentHeight()- targetPosition) < 500) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator command interrupted");
            elevator.stop();
        } else {
            System.out.println("Elevator reached target");
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
