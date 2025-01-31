package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    // Motion Magic constants (these will need tuning)
    private static final double kF = 0.05; // This is feedforward, which helps counter gravity/robot weight. Adjust based on elevator performance.
    private static final double kP = 0.1; // Provides initial correction. Too high can lead to oscillations.
    private static final double kI = 0.0; // Fine to start with, avoids integral wind-up.
    private static final double kD = 0.0; // Don't Touch unless it overshoots like crazy
    private static final int cruiseVelocity = 15000;  // Sensor units per 100ms
    private static final int acceleration = 6000;  // Sensor units per 100ms^2

//  private static final int allowableError = 500;  // Tolerable error in sensor units



  public elevator() {
      if (RobotBase.isSimulation()) {
            io = new ElevatorIOSim();  // Use simulation I/O during simulation
        } else {
            io = new ElevatorIOReal();  // Use real hardware I/O during deployment
        }
    // Initialize the elevator subsystem
    leftMotor = new TalonFX(5, "rio");  //ID 5 for left motor
    rightMotor = new TalonFX(6, "rio"); //ID 6 for right motor

    configureMotors();
  }

  @Override
    public void periodic() {
      io.updateInputs(inputs);

      // Optionally log or display on SmartDashboard for debugging
      System.out.println("Elevator Position: " + inputs.position + " meters");
      System.out.println("Elevator Velocity: " + inputs.velocity + " m/s");
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

        // Set Motion Magic PID and acceleration configs
        leftMotorConfig.Slot0.kP = kP;
        leftMotorConfig.Slot0.kI = kI;
        leftMotorConfig.Slot0.kD = kD;
        leftMotorConfig.Slot0.kV = kF;
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = acceleration;

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

    ///public void setTargetHeight(double targetPosition) {
        ///MotionMagicDutyCycle control = new MotionMagicDutyCycle(targetPosition);
        //leftMotor.setControl(control);
   // }

    //public void stop() {
        //leftMotor.set(0);
    //}

    public double getCurrentHeight() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        leftMotor.set(0.0);
    }
}
