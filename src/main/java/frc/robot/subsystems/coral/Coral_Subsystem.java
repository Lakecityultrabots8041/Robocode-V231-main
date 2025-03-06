package frc.robot.subsystems.coral;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The Coral_Subsystem handles the intake and outake functionality of the robot.
 * It uses a single motor that can run in both directions.
 */

public class Coral_Subsystem extends SubsystemBase {

    private final TalonFX coralMotor;
    
    /**
     * Creates a new Coral_Subsystem that handles both intake and outake functionality
     * with a single motor that can run in both directions.
     */
    public Coral_Subsystem() {
        // Initialize motor with ID 7
        coralMotor = new TalonFX(CoralConstants.MOTOR_ID, "rio");
        
        // Configure motor
        configureMotor();
        
        // Add to dashboard
        SmartDashboard.putData("Coral Subsystem", this);
    }
    
    /**
     * Configure the motor with appropriate settings
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Set neutral mode to brake
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limiting
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configuration
        coralMotor.getConfigurator().apply(config);
        
        Logger.recordOutput("Coral/MotorConfigured", true);
    }
    
    /**
     * Run the motor in intake direction
     * @param speed The speed to run the intake (positive value)
     */
    public void intake(double speed) {
        coralMotor.set(Math.abs(speed)); // Ensure positive value for intake
    }
    
    /**
     * Run the motor in outake direction
     * @param speed The speed to run the outake (positive value will be negated)
     */
    public void spinOut(double speed) {
        coralMotor.set(-Math.abs(speed)); // Ensure negative value for outake
    }
    
    /**
     * Set the motor to run at the specified speed
     * Positive is intake, negative is outake
     * @param speed The speed to run the motor (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        coralMotor.set(speed);
    }
    
    /**
     * Stop the motor
     */
    public void stop() {
        coralMotor.stopMotor();
    }
    
    
    public void stopAll() {
        stop();
    }
    
    
    public void stopIntake() {
        stop();
    }
    
   
    public void stopOutake() {
        stop();
    }
    
    /**
     * Get the current motor speed
     * @return The current speed (-1.0 to 1.0)
     */
    public double getSpeed() {
        return coralMotor.get();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Motor Speed", getSpeed());
        // Log data
        Logger.recordOutput("Coral/MotorSpeed", getSpeed());
        Logger.recordOutput("Coral/MotorCurrent", coralMotor.getSupplyCurrent().getValueAsDouble());
    }
}