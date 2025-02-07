package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.Arm;
import java.util.Set;

public class ArmCommand extends Command {

    private final Arm armSubsystem;
    private final double speed;

    public ArmCommand(Arm armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("ArmCommand initialized");
    }

    @Override
    public void execute() {
        armSubsystem.runArms(speed);  // Run the arms at the given speed with is is a constant 50% speed
        System.out.println("ArmCommand executing check ya arms in the console and real life! "+ speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();  // Stop motors when command ends or is interrupted
        System.out.println("ArmCommand ended. err um stopped because you let go of the button" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;  // Run until explicitly interrupted
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(armSubsystem);  // Declare that this command requires the Arm subsystem
    }
}
