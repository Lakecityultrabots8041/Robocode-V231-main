package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;



public class elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

  public elevator() {
      if (RobotBase.isSimulation()) {
            io = new ElevatorIOSim();  // Use simulation I/O during simulation
        } else {
            io = new ElevatorIOReal();  // Use real hardware I/O during deployment
        }
 
    // Initialize the elevator subsystem
    leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "rio");
    rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "rio");

    configureMotors();
  }

  @Override
    public void periodic() {
      io.updateInputs(inputs);

      // Use this to display logging to terminal for debugging

      // *** please note that this will spam the crap out of the terminal so its not really super useful but here in case you need it ***

      //System.out.println("Elevator Position: " + inputs.position + " meters");
      //System.out.println("Elevator Velocity: " + inputs.velocity + " m/s");
 
    // Record data to the log file instead of terminal. There are a ton of spam that will be recorded to the log file so be careful with this. The reason is in teleopPeriodic() we are calling get height and position method every 20ms. Meaning you will see
    // a ton of data in the log file for the elevator subsystem.
        Logger.recordOutput("Elevator/Position", inputs.position);
        Logger.recordOutput("Elevator/Velocity", inputs.velocity);
  }

    // Forward runVolts to the correct I/O implementation
    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void setTargetHeight(double targetPositionMeters) {
       io.runSetpoint(targetPositionMeters);
   }

    public void setPID(double p, double i, double d) {
      io.setPID(p, i, d);
  }

    public void stop() {
       io.stop();
   }
    private void configureMotors() {
        // Reset motor configurations
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        // Configure feedback sensor for the left motor
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.SensorToMechanismRatio = 1.0;  // Assuming 1:1, adjust if necessary
        leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // Set Motion Magic PID and acceleration configs from constants folder
        leftMotorConfig.Slot0.kP = ElevatorConstants.kP;
        leftMotorConfig.Slot0.kI = ElevatorConstants.kI;
        leftMotorConfig.Slot0.kD = ElevatorConstants.kD;
        leftMotorConfig.Slot0.kV = ElevatorConstants.kF;
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        // Apply configuration to motors
        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    
        // Invert right motor and set to follow left motor
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        // Set neutral mode to brake
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Zero the encoders at startup
        resetEncoders();
    }


    public double getCurrentHeight() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        leftMotor.set(0.0);
    }
}
