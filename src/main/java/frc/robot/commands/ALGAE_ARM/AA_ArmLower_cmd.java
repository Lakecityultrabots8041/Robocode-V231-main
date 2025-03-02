package frc.robot.commands.ALGAE_ARM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.constants.AlgaeArm_Constants;

////--------------------------------------Algae Arm Lower Command--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\

public class AA_ArmLower_cmd extends Command {

    private final Algae_ArmLift_Sub armLift;

    public AA_ArmLower_cmd(Algae_ArmLift_Sub armLiftSubsystem) {
        this.armLift = armLiftSubsystem;
        addRequirements(armLiftSubsystem);  
    }

    @Override
    public void initialize() {
        System.out.println("Algae Arm Lower Command initialized");
        armLift.setTargetHeight(AlgaeArm_Constants.ARM_LOWER_POSITION);
    }

    @Override
    public boolean isFinished() {
        return true;  
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Algae Arm Lower movement interrupted");
        } else {
            System.out.println("Algae Arm lowered to lower position");
        }
    }
}


