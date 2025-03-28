package frc.robot.commands.Unused_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ELEVATOR.MoveElevatorUpFast_cmd;
//import frc.robot.commands.CORAL.CoralGo_cmd;
//import frc.robot.constants.AlgaeArm_Constants;
//import frc.robot.constants.CoralArm_Constants;
import frc.robot.constants.Elevator_Constants;
//import frc.robot.subsystems.coral_arm.Coral_ArmLift_Sub;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;
//import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
//import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;

/**
 * A command sequence that moves the elevator to processor height
 * and then extends the arm to scoring position.
 */


public class Move_L1_Score extends SequentialCommandGroup {

    public Move_L1_Score(Elevator_Subsystem elevator, 
    Coral_Arm_Intake_Sub reverse) {

addCommands(
// First move elevator to processor height


new MoveElevatorUpFast_cmd(elevator, Elevator_Constants.L1_Bottom_Score),  

// Wait until the elevator is close to target position
new WaitUntilCommand(() -> 
(elevator.getCurrentHeight() - Elevator_Constants.L1_Bottom_Score) < 0.1),

new CA_WheelOut_cmd(reverse, 0.30),

new WaitCommand(3),
new InstantCommand(() -> reverse.stop())


// Finally extend the algae arm
//new AA_Control_cmd(algae_arm, AlgaeArm_Constants.ARM_UPPER_POSITION)
);//DO NOT USE UNLESS ALGAE ARMS ARE PUT BACK INTO COMMANDS, not ALGAE ARM commands
}


   // ELI what are you defining and calling back to Public class for?
    //public Move_L1_Score(int i, Elevator_Subsystem m_elevator) {
    //}
}
    
