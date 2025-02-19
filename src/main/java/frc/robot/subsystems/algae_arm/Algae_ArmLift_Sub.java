package frc.robot.subsystems.algae_arm;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeArm_Constants;


public class Algae_ArmLift_Sub extends SubsystemBase {

    private final TalonFX algaeArm;

    public Algae_ArmLift_Sub(int canID) {
        algaeArm = new TalonFX(AlgaeArm_Constants.MOTOR_ID, "rio");  // Use the correct CAN ID for the Kraken motor and replace canID with the number
        algaeArm.setNeutralMode(NeutralModeValue.Brake);  // Set to brake mode for precise control
        configureMotors();
    }

    public void setTargetHeight(double targetPositionMeters) {
        final MotionMagicExpoVoltage mmReq = new MotionMagicExpoVoltage(0);
        algaeArm.setControl(mmReq.withPosition(targetPositionMeters));
    }
 
    private void configureMotors() {
        
         TalonFXConfiguration AlgaeArm = new TalonFXConfiguration();
        
         // Set Motion Magic PID and acceleration configs from constants folder
         AlgaeArm.Slot0.kS = AlgaeArm_Constants.kS;
         AlgaeArm.Slot0.kV = AlgaeArm_Constants.kV;
         AlgaeArm.Slot0.kA = AlgaeArm_Constants.kA;
         AlgaeArm.Slot0.kP = AlgaeArm_Constants.kP;
         AlgaeArm.Slot0.kI = AlgaeArm_Constants.kI;
         AlgaeArm.Slot0.kD = AlgaeArm_Constants.kd; 
         AlgaeArm.MotionMagic.MotionMagicCruiseVelocity = AlgaeArm_Constants.CRUISE_VELOCITY;
         AlgaeArm.MotionMagic.MotionMagicAcceleration = AlgaeArm_Constants.ACCELERATION;
         AlgaeArm.MotionMagic.MotionMagicJerk = AlgaeArm_Constants.JERK;
         AlgaeArm.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        algaeArm.getConfigurator().apply(AlgaeArm);

     
    }
}


