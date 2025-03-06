package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
import frc.robot.subsystems.algae_arm.Algae_Subsystem;
import frc.robot.subsystems.coral.Coral_Subsystem;
import frc.robot.Constants.*;

/**
 * Factory for creating complex multi-subsystem commands for the Ultrabots robot.
 * Only coordinates between existing subsystem commands.
 */
public final class UltrabotsCommand {

    // Private constructor to prevent instantiation
    private UltrabotsCommand() {}

    //--------------------------------------------------------------------------
    // AUTONOMOUS COMMAND SEQUENCES
    //--------------------------------------------------------------------------
    
    /**
     * Complete autonomous sequence for scoring at Level 1.
     * Positions elevator and automatically shoots coral.
     */
    public static Command autonomousL1Score(Elevator_Subsystem elevator, Coral_Subsystem coral) {
        return Commands.sequence(
            // Move elevator to L1 height using existing elevator methods
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L1_BOTTOM_SCORE), elevator),
            
            // Wait until elevator is at position
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            
            // Run coral outake for a fixed duration using coral subsystem methods
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(2.0),
                  
            // Stop coral using coral subsystem methods
            Commands.runOnce(() -> coral.stop(), coral)
        ).withName("Auto Score L1");
    }
    
    /**
     * Complete autonomous sequence for scoring at Level 2.
     */
    public static Command autonomousL2Score(Elevator_Subsystem elevator, Coral_Subsystem coral) {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L2_MIDDLE_SCORE), elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(2.0),
            Commands.runOnce(() -> coral.stop(), coral)
        ).withName("Auto Score L2");
    }
    
    /**
     * Complete autonomous sequence for scoring at Level 3.
     */
    public static Command autonomousL3Score(Elevator_Subsystem elevator, Coral_Subsystem coral) {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L3_TOP_SCORE), elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(2.0),
            Commands.runOnce(() -> coral.stop(), coral)
        ).withName("Auto Score L3");
    }

    /**
     * Autonomous command for setting up at station for coral collection.
     */
    public static Command autonomousStationSetup(Elevator_Subsystem elevator) {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.STATION_LV), elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition())
        ).withName("Auto Station Setup");
    }
    
    /**
     * Autonomous command for processor height positioning.
     */
    public static Command autonomousProcessorHeight(Elevator_Subsystem elevator) {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.HOME_POSITION), elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition())
        ).withName("Auto Processor Height");
    }
    
    /**
     * Autonomous command for collecting algae.
     */
    public static Command autonomousCollectAlgae(
            Elevator_Subsystem elevator, 
            Algae_Subsystem algae) {
        
        return Commands.sequence(
            // Position elevator and algae arm
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.STATION_LV), elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.runOnce(() -> algae.setTargetHeight(AlgaeArmConstants.ARM_LOWER_POSITION), algae),
            new WaitCommand(0.5),
            
            // Run intake using algae subsystem methods
            Commands.run(() -> algae.runIntakeMotors(0.5), algae)
                .withTimeout(3.0)
                .finallyDo((interrupted) -> algae.stopIntakeMotors())
        ).withName("Auto Collect Algae");
    }
    
    //--------------------------------------------------------------------------
    // TELEOP COMMAND SEQUENCES
    //--------------------------------------------------------------------------
    
    /**
     * Initialize collection position for algae.
     */
    public static Command prepareForAlgaeCollection(
            Elevator_Subsystem elevator, 
            Algae_Subsystem algae) {
        
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.PROCESSOR_POSITION), elevator),
            new WaitCommand(0.3),
            Commands.runOnce(() -> algae.setTargetHeight(AlgaeArmConstants.ARM_LOWER_POSITION), algae)
        ).withName("Prepare for Algae Collection");
    }
    
    /**
     * Teleop command for preparing to score coral at L1.
     */
    public static Command prepareToScoreL1(Elevator_Subsystem elevator) {
        return Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L1_BOTTOM_SCORE), elevator)
            .withName("Prepare L1 Score");
    }
    
    /**
     * Teleop command for preparing to score coral at L2.
     */
    public static Command prepareToScoreL2(Elevator_Subsystem elevator) {
        return Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L2_MIDDLE_SCORE), elevator)
            .withName("Prepare L2 Score");
    }
    
    /**
     * Teleop command for preparing to score coral at L3.
     */
    public static Command prepareToScoreL3(Elevator_Subsystem elevator) {
        return Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L3_TOP_SCORE), elevator)
            .withName("Prepare L3 Score");
    }

    /**
     * Command to set to processor/home height for fast traversal.
     */
    public static Command setToProcessorHeight(Elevator_Subsystem elevator) {
        return Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.HOME_POSITION), elevator)
            .withName("Set to Processor Height");
    }

    /**
     * Complete sequence to score at L1 in teleop.
     */
    public static Command teleOpL1Score(
            Elevator_Subsystem elevator, 
            Coral_Subsystem coral, 
            double scoreDuration) {
        
        return Commands.sequence(
            prepareToScoreL1(elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(scoreDuration)
                .finallyDo((interrupted) -> coral.stop())
        ).withName("TeleOp L1 Score");
    }
    
    /**
     * Complete sequence to score at L2 in teleop.
     */
    public static Command teleOpL2Score(
            Elevator_Subsystem elevator, 
            Coral_Subsystem coral, 
            double scoreDuration) {
        
        return Commands.sequence(
            prepareToScoreL2(elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(scoreDuration)
                .finallyDo((interrupted) -> coral.stop())
        ).withName("TeleOp L2 Score");
    }
    
    /**
     * Complete sequence to score at L3 in teleop.
     */
    public static Command teleOpL3Score(
            Elevator_Subsystem elevator, 
            Coral_Subsystem coral, 
            double scoreDuration) {
        
        return Commands.sequence(
            prepareToScoreL3(elevator),
            new WaitUntilCommand(() -> elevator.isAtTargetPosition()),
            Commands.run(() -> coral.spinOut(CoralConstants.OUTTAKE_SPEED), coral)
                .withTimeout(scoreDuration)
                .finallyDo((interrupted) -> coral.stop())
        ).withName("TeleOp L3 Score");
    }
}