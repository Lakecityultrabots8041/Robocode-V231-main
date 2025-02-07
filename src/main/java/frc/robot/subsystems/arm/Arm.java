package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    private final TalonFX leftArmMotor;
    private final TalonFX rightArmMotor;

    public Arm() {
        leftArmMotor = new TalonFX(4, "rio");  // Replace where deviceID with the actual CAN ID
        rightArmMotor = new TalonFX(5, "rio"); // Replace where deviceID with the actual CAN IDS

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configurations to both motors
        leftArmMotor.getConfigurator().apply(leftMotorConfig);
        rightArmMotor.getConfigurator().apply(rightMotorConfig);

        // Set right motor to follow and invert direction
        rightArmMotor.setControl(new Follower(leftArmMotor.getDeviceID(), true));
    }

    public void runArms(double speed) {
        leftArmMotor.set(speed);
        // rightArmMotor automatically follows leftArmMotor
    }

    public void stop() {
        leftArmMotor.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("arm/LeftMotor", leftArmMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("arm/RightMotor", rightArmMotor.getPosition().getValueAsDouble());
    }
}
