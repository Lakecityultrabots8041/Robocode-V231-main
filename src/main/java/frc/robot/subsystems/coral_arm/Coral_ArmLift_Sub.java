
package frc.robot.subsystems.coral_arm;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralArm_Constants;



public class Coral_ArmLift_Sub extends SubsystemBase {

    private final TalonFX coralArm;

    public Coral_ArmLift_Sub(int canID) {
        coralArm = new TalonFX(CoralArm_Constants.MOTOR_ID, "rio");  // Use the correct CAN ID for the Kraken motor and replace canID with the number
        coralArm.setNeutralMode(NeutralModeValue.Brake);  // Set to brake mode for precise control
        configureMotors();
    }

    public void setTargetHeight(double targetPositionMeters) {
        final MotionMagicExpoVoltage mmReq = new MotionMagicExpoVoltage(0);
        coralArm.setControl(mmReq.withPosition(targetPositionMeters));
    }
 
    private void configureMotors() {
        
         TalonFXConfiguration Coral_Motor_config = new TalonFXConfiguration();
         // Set Motion Magic PID and acceleration configs from constants folder
         Coral_Motor_config.Slot0.kS = CoralArm_Constants.kS;
         Coral_Motor_config.Slot0.kV = CoralArm_Constants.kV;
         Coral_Motor_config.Slot0.kA = CoralArm_Constants.kA;
         Coral_Motor_config.Slot0.kP = CoralArm_Constants.kP;
         Coral_Motor_config.Slot0.kI = CoralArm_Constants.kI;
         Coral_Motor_config.Slot0.kD = CoralArm_Constants.kd; 
         Coral_Motor_config.MotionMagic.MotionMagicCruiseVelocity = CoralArm_Constants.CRUISE_VELOCITY;
         Coral_Motor_config.MotionMagic.MotionMagicAcceleration = CoralArm_Constants.ACCELERATION;
         Coral_Motor_config.MotionMagic.MotionMagicJerk = CoralArm_Constants.JERK;
         Coral_Motor_config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

         coralArm.getConfigurator().apply(Coral_Motor_config);

     
    }
}

