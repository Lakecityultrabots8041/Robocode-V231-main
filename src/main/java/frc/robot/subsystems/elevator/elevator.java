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

public class Elevator extends SubsystemBase {

    public enum ElevatorHeight {
        STOWED,         // Bottom/stowed position
        CORAL_PICKUP,   // Coral intake position
        FEEDER,         // Human player station pickup
        ALGAE_L1,       // Bottom scoring (currently algae but intended for coral)
        ALGAE_L2,       // Middle scoring (currently algae but intended for coral)
        ALGAE_L3,       // Top scoring (currently algae but intended for coral)
        NET_SCORE,      // Net scoring position
        PROCESSOR,      // Transfer/processor position
        CARRY           // Safe carrying position
    }

    private ElevatorHeight currentTarget = ElevatorHeight.STOWED;
    private final ElevatorIO io;
    private final ElevatorConstants.IOInputs inputs = new ElevatorConstants.IOInputs();

    // Keep the motor objects only for configuration and encoder reading.
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public Elevator() {
        // Use simulation or real IO based on the runtime environment.
        if (RobotBase.isSimulation()) {
            io = new ElevatorIOSim();
        } else {
            io = new ElevatorIOReal();
        }
      
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "rio");
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "rio");

        configureMotors();
    }

    /**
     * Sets the target height using a preset defined by the ElevatorHeight enum.
     * This method uses a switch-case to determine the target height in meters (from ElevatorConstants)
     * and then delegates to the double-based method, which passes the setpoint to the IO layer.
     */
    public void setTargetHeight(ElevatorHeight height) {
        currentTarget = height;
        double targetPosition;
        switch (height) {
            case STOWED:
                targetPosition = ElevatorConstants.STOWED_HEIGHT;
                break;
            case CORAL_PICKUP:
                targetPosition = ElevatorConstants.CORAL_PICKUP_HEIGHT;
                break;
            case FEEDER:
                targetPosition = ElevatorConstants.FEEDER_HEIGHT;
                break;
            case ALGAE_L1:
                targetPosition = ElevatorConstants.ALGAE_L1_HEIGHT;
                break;
            case ALGAE_L2:
                targetPosition = ElevatorConstants.ALGAE_L2_HEIGHT;
                break;
            case ALGAE_L3:
                targetPosition = ElevatorConstants.ALGAE_L3_HEIGHT;
                break;
            case NET_SCORE:
                targetPosition = ElevatorConstants.NET_SCORE_HEIGHT;
                break;
            case PROCESSOR:
                targetPosition = ElevatorConstants.PROCESSOR_HEIGHT;
                break;
            case CARRY:
                targetPosition = ElevatorConstants.CARRY_HEIGHT;
                break;
            default:
                targetPosition = ElevatorConstants.STOWED_HEIGHT;
                break;
        }
        // Delegate to the double-based method so that conversion happens only in the IO layer.
        setTargetHeight(targetPosition);
    }

    /**
     * Sets the target height in meters.
     * This method passes the setpoint to the IO layer, which converts meters to rotations
     * and sends the Motion Magic command.
     */
    public void setTargetHeight(double targetPositionMeters) {
        io.runSetpoint(targetPositionMeters);
    }

    public void holdPosition() {
        io.runSetpoint(getCurrentHeight());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Elevator/Position", inputs.position);
        Logger.recordOutput("Elevator/Velocity", inputs.velocity);
        Logger.recordOutput("Elevator/TargetHeight", currentTarget.toString());
    }

    // Delegate manual voltage control to IO.
    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    // Delegate dynamic PID updates to IO.
    public void setPID(double p, double i, double d) {
        io.setPID(p, i, d);
    }

    // Delegate stopping to IO.
    public void stop() {
        io.stop();
    }

    /**
     * Configures the motor controllers with Motion Magic and PID settings.
     * Encoder reset is performed here to zero the left encoder.
     */
    private void configureMotors() {
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.SensorToMechanismRatio = 1.0;
        leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        leftMotorConfig.Slot0.kP = ElevatorConstants.kP;
        leftMotorConfig.Slot0.kI = ElevatorConstants.kI;
        leftMotorConfig.Slot0.kD = ElevatorConstants.kD;
        leftMotorConfig.Slot0.kV = ElevatorConstants.kF;
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    
        // Right motor follows the left motor (inverted).
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Zero the encoder on the left motor.
        resetEncoders();
    }

    /**
     * Returns the current elevator height (using the left motor encoder).
     */
    public double getCurrentHeight() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Resets the encoder by zeroing the left motor encoder.
     */
    public void resetEncoders() {
        leftMotor.set(0.0);
    }

    /**
     * Returns the current target preset.
     */
    public ElevatorHeight getCurrentTarget() {
        return currentTarget;
    }
}
