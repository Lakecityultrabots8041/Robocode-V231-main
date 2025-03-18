package frc.robot.subsystems.coral_arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralArm_Constants;


public class Coral_Arm_Intake_Sub extends SubsystemBase {
    //private final VictorSPX intakeMotor;
    private final TalonFX intakeMotor;

    public Coral_Arm_Intake_Sub() {
        intakeMotor = new TalonFX(CoralArm_Constants.MOTOR_ID);
    }

    public void intake(double speed) {
        //intakeMotor.set(ControlMode.PercentOutput, 0.30); // Run the motor at NOT full speed to intake
        intakeMotor.set(0.15);
    }

    public void stop() {
        //intakeMotor.set(ControlMode.PercentOutput, 0); // Stop the motor
        intakeMotor.set(0);
    }

    public void reverse(double speed) {
        //intakeMotor.set(ControlMode.PercentOutput, -0.30); // Run the motor in reverse to expel
        intakeMotor.set(-speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}