package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.IOInputs;


public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;


    // Convert rotations to meters (adjust these values based on your elevator's gearing)
    private static final double ROTATIONS_TO_METERS = Math.PI * 0.0254 * 1.751; // Example: 1.751" diameter pulley


  //private static final double ENCODER_TO_METERS = 1.0;  // Conversion factor from sensor units to meters

    public ElevatorIOReal() {

        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "rio");
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "rio");
        configureMotors();

    }

    private void configureMotors() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
             
        // Motion Magic configuration
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        // PID configuration
        motorConfig.Slot0.kP = ElevatorConstants.kP;
        motorConfig.Slot0.kI = ElevatorConstants.kI;
        motorConfig.Slot0.kD = ElevatorConstants.kD;
        motorConfig.Slot0.kV = ElevatorConstants.kF;
        motorConfig.Slot0.kS = ElevatorConstants.kS; // Static friction compensation
        motorConfig.Slot0.kG = ElevatorConstants.kG; // Gravity compensation if needed

        // Motor output configuration
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // does brake get applied?
        motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001; // Smaller deadband for more precise control

        // Apply configuration to motors
        leftMotor.getConfigurator().apply(motorConfig);
        rightMotor.getConfigurator().apply(motorConfig);

         // Set right motor to follow left motor (inverted)
        rightMotor.setControl(new com.ctre.phoenix6.controls.Follower(leftMotor.getDeviceID(), true));
    
    }

    @Override
    public void updateInputs(IOInputs inputs) {
        inputs.position = leftMotor.getPosition().getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.velocity = leftMotor.getVelocity().getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.appliedVoltsLeader = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.appliedVoltsFollower = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentLeader = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.supplyCurrentFollower = rightMotor.getSupplyCurrent().getValueAsDouble();
    }

    // ------------------------------------------------------ Run Setpoint Conversion of Motion magic to encoder units------------------------------------------------------
    @Override
    public void runSetpoint(double targetPositionMeters) {
        // Convert meters to rotations
        double targetRotations = targetPositionMeters / ROTATIONS_TO_METERS;
        
        // Create Motion Magic control request with position target
        MotionMagicVoltage motionMagic = new MotionMagicVoltage(targetRotations)
            .withSlot(0);  // Use slot 0 configuration        
        leftMotor.setControl(motionMagic);
    }

    @Override
    public void setPID(double p, double i, double d) {
        Slot0Configs pidConfigs = new Slot0Configs();
        pidConfigs.kP = p;
        pidConfigs.kI = i;
        pidConfigs.kD = d;
        // Only update PID configs, leaving other configurations unchanged
        leftMotor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor(); // will this stop both motors if the right is following?
    }

    @Override
    public void runVolts(double volts) {
        // This might be useful for testing, but prefer using Motion Magic for normal operation
        leftMotor.setVoltage(volts);
    }
}