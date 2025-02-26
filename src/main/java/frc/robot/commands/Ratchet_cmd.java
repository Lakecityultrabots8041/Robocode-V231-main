package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import frc.robot.subsystems.Ratchet_Climber.Ratchet_Subsystem;
import frc.robot.constants.Climber_Ratchet_Constants;


/*
 * okay, here me out
 * I want to create a boolean called isEngaged that will send the servo to 1 of 2 positions based on
 * its status
 * 
 * if isEngaged is true, then the servo will be set to the off position and isEngaged will be set to false
 * if isEngaged is false, then the servo will be set to the on position and isEngaged will be set to true
 * 
 * But I don't know if I can use a boolean inside of a command like this and have it stick...
 * 
 * So don't be surprised if this doesn't work
 */
public class Ratchet_cmd extends Command {

    public final Ratchet_Subsystem ratchetSubsystem;
    //public final double position;
    public static boolean isEngaged = false;

    public Ratchet_cmd(Ratchet_Subsystem ratchetSubsystem) {
        this.ratchetSubsystem = ratchetSubsystem;
        //this.position = position;
        addRequirements(ratchetSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Ratchet initialized");
    }

    @Override
    public void execute() {        
        /* 
        This is the "offending code"
        if(isEngaged){
            ratchetSubsystem.setPosition(Climber_Ratchet_Constants.Ratchet_Off);
            isEngaged = false;
        }
        else {
            ratchetSubsystem.setPosition(Climber_Ratchet_Constants.Ratchet_On);
            isEngaged = true;
        }
            */
        //ratchetSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return false;  // Run until explicitly interrupted
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(ratchetSubsystem);  // Declare that this command requires the Ratchet subsystem
    }

    

}


