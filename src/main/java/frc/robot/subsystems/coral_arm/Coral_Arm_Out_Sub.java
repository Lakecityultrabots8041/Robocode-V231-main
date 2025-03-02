package frc.robot.subsystems.coral_arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralArm_Constants;


public class Coral_Arm_Out_Sub extends SubsystemBase {
    //private final VictorSPX intakeMotor;
    private final TalonFX OutMotor;

    public Coral_Arm_Out_Sub() {
        OutMotor = new TalonFX(CoralArm_Constants.MOTOR_ID);
    }

    public void Out(double speed) {
        //intakeMotor.set(ControlMode.PercentOutput, 0.30); // Run the motor at NOT full speed to intake
        OutMotor.set(-0.15);
    }

    public void stop() {
        //intakeMotor.set(ControlMode.PercentOutput, 0); // Stop the motor
        OutMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

