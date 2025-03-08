package frc.robot.commands.Auton_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
import frc.robot.commands.ELEVATOR.SetElevatorLevel;
import frc.robot.commands.Unused_Commands.CA_WheelOut_cmd;
import frc.robot.constants.Elevator_Constants;

/**
 * Autonomous command to move elevator to L1 score position and eject note
 */
public class AUTON_CMD extends SequentialCommandGroup {

    public AUTON_CMD(Elevator_Subsystem elevator, Coral_Arm_Intake_Sub coralIntake) {
        addCommands(
            // Debug message - starting autonomous sequence
            new InstantCommand(() -> System.out.println("Starting L1 Auto Sequence")),
            
            // First set the elevator target to L1 position
            new SetElevatorLevel(elevator, Elevator_Constants.L2_Middle_Score),
            
            // Debug message - elevator target set
            new InstantCommand(() -> System.out.println("Elevator target set to L1, current height: " + elevator.getCurrentHeight())),
            
            // Wait until elevator is near the target position
            new WaitUntilCommand(() -> {
                double currentHeight = elevator.getCurrentHeight();
                double targetHeight = Elevator_Constants.L1_Bottom_Score;
                double difference = Math.abs(currentHeight - targetHeight);
                System.out.println("Waiting for elevator, current height: " + currentHeight + ", difference: " + difference);
                return difference < 0.1;
            }),
            
            // Debug message - elevator in position
            new InstantCommand(() -> System.out.println("ELEVATOR IN POSITION - ACTIVATING CORAL ARM")),
            
            // Run coral outake using RunCommand with a timeout
            // Note: Using RunCommand instead of your CA_WheelOut_cmd since CA_WheelOut_cmd
            // negates the speed value internally (and we don't want double negation)
            new RunCommand(() -> {
                // Use the positive speed since we know the coral outake needs positive
                // values based on our testing
                System.out.println("RUNNING CORAL OUTAKE");
                coralIntake.reverse(0.30);
            }, coralIntake).withTimeout(1.5),
            
            // Debug message - stopping coral
            new InstantCommand(() -> System.out.println("STOPPING CORAL ARM")),
            
            // Stop the coral intake
            new InstantCommand(() -> coralIntake.stop()),
            
            // Debug message - sequence complete
            new InstantCommand(() -> System.out.println("L1 Auto Sequence Complete"))
        );
    }
    
    // Factory method to create this command without needing to use 'new'
    public static Command create(Elevator_Subsystem elevator, Coral_Arm_Intake_Sub coralIntake) {
        return new AUTON_CMD(elevator, coralIntake);
    }
}