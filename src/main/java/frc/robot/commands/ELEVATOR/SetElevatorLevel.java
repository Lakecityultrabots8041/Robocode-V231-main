package frc.robot.commands.ELEVATOR;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

public class SetElevatorLevel extends Command {


    //stores reference to the elevator subsystem and the target height
    private final Elevator_Subsystem elevator;
    //stores the target height
    private final double targetHeight;

    //constructor that takes in the elevator subsystem and the target height and will add the elevator subsystem as a requirement
    public SetElevatorLevel(Elevator_Subsystem elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator); /*Declare that this command requires the Elevator subsystem
                                     This is important as it tells the scheduler that the Elevator subsystem is in use and nothing else can run with elevator subsystem
                                     */
    }

    /*  Below are the methods that are called when the command is run, 
    *   also known as the lifecycle method of a command. Each command has 
    *   a lifecycle method and usually has an execute method but not always the case
    */

    @Override
    public void initialize() {
        elevator.setTargetHeight(targetHeight);
    }
    @Override
    public boolean isFinished() {
        return true; // Run onces so it is finished
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator movement interrupted");
        } else {
            System.out.println("Elevator at target height");
        }
    }
}
