package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral_arm.Coral_ArmLift_Sub;

public class CA_ctrl_cmd extends Command {

    private final Coral_ArmLift_Sub armLift;
    private final double targetHeight;

    public CA_ctrl_cmd(Coral_ArmLift_Sub armLiftSubsystem, double targetHeight) {
        this.armLift = armLiftSubsystem;
        this.targetHeight = targetHeight;
        addRequirements(armLiftSubsystem); 
    } 
    
    @Override
    public void initialize() {
        
        System.out.println("LiftArmCommand initialized");
        armLift.setTargetHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return true;  
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Arm movement interrupted");
        } else {
            System.out.println("Arm target height set");
        }
        // Don't stop the arm - let Motion Magic continue to the target
    }

}
