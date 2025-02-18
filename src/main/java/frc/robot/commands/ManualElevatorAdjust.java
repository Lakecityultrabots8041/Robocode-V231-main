package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ManualElevatorAdjust extends Command {
    private final Elevator Elevator;
    private final double adjustmentSpeed;
    private static final double MAX_HEIGHT = 2.5; // Maximum safe height in meters
    private static final double MIN_HEIGHT = 0.0; // Minimum safe height in meters


    public ManualElevatorAdjust(Elevator elevator, double adjustmentSpeed) {
        this.Elevator = elevator;
        this.adjustmentSpeed = adjustmentSpeed;
        addRequirements(elevator);
    }
    @Override
    public void execute() {
        double currentHeight = Elevator.getCurrentHeight();
        
        // Safety checks
        if ((currentHeight >= MAX_HEIGHT && adjustmentSpeed > 0) ||
            (currentHeight <= MIN_HEIGHT && adjustmentSpeed < 0)) {
            Elevator.stop();
            return;
        }

        Elevator.runVolts(adjustmentSpeed * 12.0); // Scale to voltage
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}