package frc.robot.subsystems.Ratchet_Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.Climber_Ratchet_Constants;



public class Climber_Subsystem {

    private final TalonFX climber;

    public Climber_Subsystem () {
        climber = new TalonFX(Climber_Ratchet_Constants.ClimberMotorID, "rio");  // Use the correct CAN ID for the Kraken motor and replace canID with the number
        configureMotors();
    }
     private void configureMotors() {
        
         TalonFXConfiguration Climber = new TalonFXConfiguration();
        
         // Set Motion Magic PID and acceleration configs from constants folder
         Climber.Slot0.kS = Climber_Ratchet_Constants.kS;
         Climber.Slot0.kV = Climber_Ratchet_Constants.kV;
         Climber.Slot0.kP = Climber_Ratchet_Constants.kP;
         Climber.Slot0.kI = Climber_Ratchet_Constants.kI;
         Climber.Slot0.kD = Climber_Ratchet_Constants.kD; 
         Climber.Slot0.kG = Climber_Ratchet_Constants.kG;

         Climber.MotorOutput.NeutralMode = NeutralModeValue.Brake;;  // Set to brake mode for precise control

        climber.getConfigurator().apply(Climber);

     
    }
}

