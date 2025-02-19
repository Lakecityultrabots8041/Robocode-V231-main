package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Elevator_Constants;
import org.littletonrobotics.junction.Logger;


public class Elevator_Subsystem extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;


    // Creates a visualizer for the elevator for smartdashboard to log data for elevator
    private final ElevatorVisualizer visualizer;

    
  public Elevator_Subsystem() {
     
    // Initialize the elevator subsystem
    leftMotor = new TalonFX(Elevator_Constants.LEFT_MOTOR_ID, "rio");
    rightMotor = new TalonFX(Elevator_Constants.RIGHT_MOTOR_ID, "rio");

    // Initialize the visualizer for the elevator
    visualizer = new ElevatorVisualizer("elevator", edu.wpi.first.wpilibj.util.Color.kCyan);
    
    configureMotors();
  }

    public void setTargetHeight(double targetPositionMeters) {
       final MotionMagicExpoVoltage mmReq = new MotionMagicExpoVoltage(0);
       leftMotor.setControl(mmReq.withPosition(targetPositionMeters));
   }
   
    private void configureMotors() {
        // Reset motor configurations
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        // Configure feedback sensor for the left motor
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();

        feedbackConfig.SensorToMechanismRatio = 1.0;  // Assuming 1:1, adjust if necessary
        leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // Slot0 PID configuration
        leftMotorConfig.Slot0.kS = Elevator_Constants.kS;
        leftMotorConfig.Slot0.kV = Elevator_Constants.kV;
        leftMotorConfig.Slot0.kA = Elevator_Constants.kA;
        leftMotorConfig.Slot0.kP = Elevator_Constants.kP;
        leftMotorConfig.Slot0.kI = Elevator_Constants.kI;
        leftMotorConfig.Slot0.kD = Elevator_Constants.kD;
        
        /* 
        *  Slot0 controls "how tightly" the system follows the target
        *   MotionMagic controls "how smoothly" the target moves from point A to point B
        */ 

        //--------------------Motion Magic Configurations--------------------

        leftMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12; //Do we need the Expor prarameters if we already have set regular Slot0 pid values?
        leftMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;  //Do we need the Expor prarameters if we already have set regular Slot0 pid values?
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator_Constants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = Elevator_Constants.ACCELERATION;

        
        // Apply's brake mode to motors when not moving
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            

        // Apply new configuration to motors from the configure motors 
        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    
        // Invert right motor and set to follow left motor
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        // Zero the encoders at startup
        resetEncoders();
    }


    public double getCurrentHeight() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        leftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // Log data for debugging and dashboards
        Logger.recordOutput("Elevator/Position", getCurrentHeight());
        Logger.recordOutput("Elevator/Velocity", leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Elevator/LeftMotorCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Elevator/RightMotorCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
        
        // Call the visualizer to update the height of the elevator periodically
        visualizer.update(edu.wpi.first.units.Units.Meters.of(getCurrentHeight()));
    }
}

