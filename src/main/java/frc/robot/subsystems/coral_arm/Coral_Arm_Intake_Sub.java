package frc.robot.subsystems.coral_arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralArm_Constants;


public class Coral_Arm_Intake_Sub extends SubsystemBase {
    private final VictorSPX intakeMotor;

    public Coral_Arm_Intake_Sub() {
        intakeMotor = new VictorSPX(CoralArm_Constants.intake_motor_ID);
    }

    public void intake(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, 0.20); // Run the motor at NOT full speed to intake
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0); // Stop the motor
    }

    public void reverse(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, -0.20); // Run the motor in reverse to expel
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}