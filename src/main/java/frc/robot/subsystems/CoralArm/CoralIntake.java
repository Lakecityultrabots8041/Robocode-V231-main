package frc.robot.subsystems.CoralArm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralIntake extends SubsystemBase {
    private final VictorSPX intakeMotor;

    public CoralIntake(int motorPort) {
        intakeMotor = new VictorSPX(motorPort);
    }

    public void intake() {
        intakeMotor.set(ControlMode.PercentOutput, 1.0); // Run the motor at full speed to intake
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0); // Stop the motor
    }

    public void reverse() {
        intakeMotor.set(ControlMode.PercentOutput, -1.0); // Run the motor in reverse to expel
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}