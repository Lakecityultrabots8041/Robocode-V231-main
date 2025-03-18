package frc.robot.commands.Unused_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import frc.robot.constants.AlgaeArm_Constants;
import frc.robot.constants.CoralArm_Constants;
import frc.robot.constants.Elevator_Constants;
import frc.robot.subsystems.coral_arm.Coral_ArmLift_Sub;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;

/**
 * A command sequence that moves the elevator to processor height
 * and then extends the arm to scoring position.
 */


public class Move_L3_Score extends SequentialCommandGroup {

    public Move_L3_Score(Elevator_Subsystem elevator, 
    Coral_Arm_Intake_Sub reverse) {

addCommands(
// First move elevator to processor height


//new SetElevatorLevel(elevator, Elevator_Constants.L3_TOP_Score),  

// Wait until the elevator is close to target position
new WaitUntilCommand(() -> 
Math.abs(elevator.getCurrentHeight() - Elevator_Constants.L3_TOP_Score) < 0.1),

new CA_WheelOut_cmd(reverse, 0.30),

new WaitCommand(3),
new InstantCommand(() -> reverse.stop())

// Finally extend the algae arm
//new AA_Control_cmd(algae_arm, AlgaeArm_Constants.ARM_UPPER_POSITION)
);//DO NOT USE UNLESS ALGAE ARMS ARE PUT BACK INTO COMMANDS, not ALGAE ARM commands
}
}
    
    