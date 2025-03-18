package frc.robot.commands.Auton_Commands;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
//import frc.robot.commands.ALGAE_ARM.*;
import frc.robot.commands.Unused_Commands.CA_WheelOut_cmd;
import frc.robot.constants.*;
//import frc.robot.commands.ELEVATOR.SetElevatorLevel;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ELEVATOR.MoveElevatorUpFast_cmd;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public final class Auton_L1_cmd extends SequentialCommandGroup {

    private Auton_L1_cmd() {}

    public Auton_L1_cmd(Elevator_Subsystem elevator, 
    Coral_Arm_Intake_Sub reverse) {

        addCommands(
// First move elevator to processor height


new MoveElevatorUpFast_cmd(elevator, Elevator_Constants.L1_Bottom_Score),  

// Wait until the elevator is close to target position
new WaitUntilCommand(() -> 
(elevator.getCurrentHeight() - Elevator_Constants.L1_Bottom_Score) < 0.1),

new CA_WheelOut_cmd(reverse, 0.30),

new WaitCommand(3),
new InstantCommand(() -> reverse.stop()));
    }

}
