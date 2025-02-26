package frc.robot.subsystems.Ratchet_Climber;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ratchet_Subsystem extends SubsystemBase {
    
    
    private final Servo servo;

     public Ratchet_Subsystem() {
        servo = new Servo(0);
    }

    public void setPosition(double position) {
        servo.set(position);
    }
    public void Close(double closedPosition) {
        // Position should be between 0.0 (0 degrees) and 1.0 (180 degrees)
        servo.set(0);
    }

    public void Open(double openPosition) {
        // Position should be between 0.0 (0 degrees) and 1.0 (180 degrees)
        servo.set(0.12);
    }

    public double getPOS() {
        return servo.get();
    }
}
