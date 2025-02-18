package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorLevel extends Command {

    private final Elevator elevator;
    private final double targetHeight;
    private final double tolerance = 0.05; // in meters

    public SetElevatorLevel(Elevator elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetHeight(targetHeight);
    }
    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentHeight() - targetHeight) < tolerance; // this seems like very unsafe code but i not sure right now what else to do. if the tolerance is too small the elevator will never stop so kind of sketch
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator movement interrupted");
        }
    }
}