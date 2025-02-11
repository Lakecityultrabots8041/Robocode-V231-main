package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIOs {

    private final TalonFX leftMotor = new TalonFX(5, "rio");  // Set correct CAN ID
    private final TalonFX rightMotor = new TalonFX(6, "rio"); // Set correct CAN ID

    private static final double ENCODER_TO_METERS = 1.0;  // Conversion factor from sensor units to meters

    public ElevatorIOReal() {
        // Make the right motor follow the left motor
        rightMotor.setControl(new com.ctre.phoenix6.controls.Follower(leftMotor.getDeviceID(), true));
        // Apply initial motor configuration
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotor.getConfigurator().apply(motorConfig);
        rightMotor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = leftMotor.getPosition().getValueAsDouble() * ENCODER_TO_METERS;  // This could help reset when new position is set
        inputs.velocity = leftMotor.getVelocity().getValueAsDouble() * ENCODER_TO_METERS;
        inputs.appliedVoltsLeader = leftMotor.getSupplyVoltage().getValueAsDouble();
        inputs.appliedVoltsFollower = rightMotor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrentLeader = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentFollower = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void runSetpoint(double targetPositionMeters) {
        // Convert meters to sensor units if necessary (assuming 1:1 for now)
        double targetPositionUnits = targetPositionMeters / ENCODER_TO_METERS;
        MotionMagicDutyCycle control = new MotionMagicDutyCycle(targetPositionUnits);
        leftMotor.setControl(control);
    }

    @Override
    public void runVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void setPID(double p, double i, double d) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        leftMotor.getConfigurator().apply(config);
    }

    @Override
    public void stop() {
        leftMotor.set(0);
    }
}
