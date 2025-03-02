package frc.robot.commands.Unused_Commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;

////--------------------------------------Coral Intake Command--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\


public class CA_WheelOut_cmd extends Command {

    private final Coral_Arm_Intake_Sub armSubsystem;
    private final double speed;

    public CA_WheelOut_cmd(Coral_Arm_Intake_Sub armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CoralEject initialized");
    }

    @Override
    public void execute() {
        armSubsystem.reverse(-speed);  // Run the arms at the given speed with is is a constant 50% speed
        System.out.println("CoralEject executing check ya arms in the console and real life! "+ speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();  // Stop motors when command ends or is interrupted
        System.out.println("CoralEject ended. err um stopped because you let go of the button" + interrupted);
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

