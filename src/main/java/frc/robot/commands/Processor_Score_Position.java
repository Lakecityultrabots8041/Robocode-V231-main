package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.AlgaeArm_Constants;
import frc.robot.constants.Elevator_Constants;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

/**
 * A command sequence that moves the elevator to processor height
 * and then extends the arm to scoring position.
 */
public class Processor_Score_Position extends SequentialCommandGroup {
    
    public Processor_Score_Position(Elevator_Subsystem elevator, Algae_ArmLift_Sub arm) {
        addCommands(
            // First move elevator to processor height
            new SetElevatorLevel(elevator, Elevator_Constants.Processor_Position),
            
            // Wait until the elevator is close to target position
            new WaitUntilCommand(() -> 
                Math.abs(elevator.getCurrentHeight() - Elevator_Constants.Processor_Position) < 0.05),
            
            // Then extend the arm to scoring position
            new Lift_AlgaeArm_Command(arm, AlgaeArm_Constants.ARM_UPPER_POSITION)
        );
    }
}