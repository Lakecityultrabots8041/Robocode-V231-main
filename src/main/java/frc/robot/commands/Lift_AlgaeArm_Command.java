package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;

public class Lift_AlgaeArm_Command extends Command {

    private final Algae_ArmLift_Sub armLift;
    private final double targetHeight;

    public Lift_AlgaeArm_Command(Algae_ArmLift_Sub armLiftSubsystem, double targetHeight) {
        this.armLift = armLiftSubsystem;
        this.targetHeight = targetHeight;
        addRequirements(armLiftSubsystem);  //Ensures that only one command can use the subsystem at a time
    }


    @Override
    public void initialize() {
        
        System.out.println("LiftArmCommand initialized");
        armLift.setTargetHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return true;  // Run until explicitly interrupted
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