package frc.robot.commands.Unused_Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import frc.robot.constants.AlgaeArm_Constants;
import frc.robot.constants.CoralArm_Constants;
import frc.robot.constants.Elevator_Constants;
import frc.robot.subsystems.coral_arm.Coral_ArmLift_Sub;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

/**
 * A command sequence that moves the elevator to processor height
 * and then extends the arm to scoring position.
 */
//Change values from L3 to L2

public class Move_L2_Score extends SequentialCommandGroup {

    public Move_L2_Score(Elevator_Subsystem elevator, 
    Coral_ArmLift_Sub coral_arm,  
    Algae_ArmLift_Sub algae_arm) {

addCommands(

//new SetElevatorLevel(elevator, Elevator_Constants.L2_Middle_Score),  

// Wait until the elevator is close to target position
new WaitUntilCommand(() -> 
Math.abs(elevator.getCurrentHeight() - Elevator_Constants.L2_Middle_Score) < 0.1),

// Then extend the coral arm to scoring position
new CA_ctrl_cmd(coral_arm, CoralArm_Constants.ARM_UPPER_POSITION),

// Wait until coral arm is in position
new WaitUntilCommand(() -> 
Math.abs(coral_arm.getCurrentHeight() - CoralArm_Constants.ARM_UPPER_POSITION) < 0.05)

// Finally extend the algae arm
//new AA_Control_cmd(algae_arm, AlgaeArm_Constants.ARM_UPPER_POSITION)
);//DO NOT USE UNLESS ALGAE ARMS ARE PUT BACK INTO COMMANDS, not ALGAE ARM commands
}
}
    
    