package frc.robot.subsystems.algae_arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;

public class Algae_Subsystem extends SubsystemBase {

    // Define lift motor
    private final TalonFX liftMotor;
    
    // Define Intake/eject motors
    private final WPI_VictorSPX leftIntakeMotor;
    private final WPI_VictorSPX rightIntakeMotor;

    /**
     * Constructs a new AlgaeArmSubsystem.
     */
    public Algae_Subsystem() {

        // Initialize lift motor
        liftMotor = new TalonFX(AlgaeArmConstants.MOTOR_ID, "rio");
        
        // Initialize intake motors with correct CAN IDs
        leftIntakeMotor = new WPI_VictorSPX(4); // call in contants file
        rightIntakeMotor = new WPI_VictorSPX(2); // call in contants file
        
        // Build constructor for motors
        configureLiftMotor();
        configureIntakeMotors();
    }

    //--------------------------------------------------------------------------
    // CONFIGURATION METHODS
    //--------------------------------------------------------------------------
    
    /**
     * Configure the lift motor with the appropriate settings.
     */

    private void configureLiftMotor() {
        TalonFXConfiguration liftConfig = new TalonFXConfiguration();
        
        // Set Motion Magic PID and acceleration configs from constants folder
        liftConfig.Slot0.kS = AlgaeArmConstants.kS;
        liftConfig.Slot0.kV = AlgaeArmConstants.kV;
        liftConfig.Slot0.kA = AlgaeArmConstants.kA;
        liftConfig.Slot0.kP = AlgaeArmConstants.kP;
        liftConfig.Slot0.kI = AlgaeArmConstants.kI;
        liftConfig.Slot0.kD = AlgaeArmConstants.kD; 
        liftConfig.Slot0.kG = AlgaeArmConstants.kG;
        liftConfig.MotionMagic.MotionMagicCruiseVelocity = AlgaeArmConstants.CRUISE_VELOCITY;
        liftConfig.MotionMagic.MotionMagicAcceleration = AlgaeArmConstants.ACCELERATION;
        liftConfig.MotionMagic.MotionMagicJerk = AlgaeArmConstants.JERK;
        liftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        liftMotor.getConfigurator().apply(liftConfig);
    }

    /**
     * Configure the intake motors with the appropriate settings.
     */
    private void configureIntakeMotors() {
        // Set motors to Brake mode for precise control
        leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
        rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    //--------------------------------------------------------------------------
    // LIFT METHODS
    //--------------------------------------------------------------------------
    
    /**
     * Sets the target height for the lift arm.
     * 
     * @param targetPositionMeters The target position in meters.
     */
    public void setTargetHeight(double targetPositionMeters) {
        final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
        liftMotor.setControl(mmReq.withPosition(targetPositionMeters));
    }

    //--------------------------------------------------------------------------
    // INTAKE/EJECT METHODS
    //--------------------------------------------------------------------------
    
    /**
     * Runs both arm motors at the specified speed.
     * 
     * @param speed The speed to run the motors at (-1.0 to 1.0).
     */
    public void runIntakeMotors(double speed) {
        leftIntakeMotor.set(ControlMode.PercentOutput, speed);
        rightIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs the intake at a preset slow speed.
     */
    public void runIntakeSlow() {
        leftIntakeMotor.set(ControlMode.PercentOutput, 0.20);
        rightIntakeMotor.set(ControlMode.PercentOutput, 0.20);
    }


     /**
     * Runs the eject operation at the specified speed.
     * 
     * @param speed The positive speed value (will be made negative internally)
     */
    public void runEject(double speed) {

        // Documentation for this is confusing, but this seems like it should work?
        double positiveSpeed = Math.abs(speed);
        leftIntakeMotor.set(-positiveSpeed);
        rightIntakeMotor.set(-positiveSpeed);
    }
    
    
     // Runs the eject at a preset speed. I am leaving this hear since it was nice if we had to make changes to the speed on the fly. 
    
    public void runEjectDefault() {
        leftIntakeMotor.set(ControlMode.PercentOutput, -0.50);
        rightIntakeMotor.set(ControlMode.PercentOutput, -0.50);
    }

    /**
     * Stops all intake motors.
     */
    public void stopIntakeMotors() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }

    //--------------------------------------------------------------------------
    // COMMAND FACTORIES
    //--------------------------------------------------------------------------
    
    /**
     * Creates a command to set the arm to a specific height.
     * 
     * @param targetHeight The target height to set the arm to.
     * @return A command that sets the arm to the target height.
     */
    public Command setArmHeightCommand(double targetHeight) {
        return this.runOnce(() -> {
            setTargetHeight(targetHeight);
            System.out.println("Setting arm height to " + targetHeight);
        }).withName("Arm Height " + targetHeight);
    }

    /**
     * Creates a command to lower the arm to its lowest position.
     * 
     * @return A command that lowers the arm.
     */
    public Command lowerArmCommand() {
        return this.runOnce(() -> {
            setTargetHeight(AlgaeArmConstants.ARM_LOWER_POSITION);
            System.out.println("Lowering arm to " + AlgaeArmConstants.ARM_LOWER_POSITION);
        }).withName("Lower Arm");
    }

    /**
     * Creates a command to run the intake.
     * 
     * @param speed The speed to run the intake at (positive for intake).
     * @return A command that runs the intake continuously until interrupted.
     */
    public Command intakeCommand(double speed) {
        return this.run(() -> {
            runIntakeMotors(speed);
            System.out.println("Running intake at " + speed);
        }).withName("Intake").finallyDo((interrupted) -> {
            stopIntakeMotors();
            System.out.println("Intake stopped. Interrupted: " + interrupted);
        });
    }

    /**
     * Creates a command to run the eject.
     * 
     * @param speed The speed to run the eject at (negative for eject).
     * @return A command that runs the eject continuously until interrupted.
     */

    public Command ejectCommand(double speed) {
        return this.run(() -> {
            runEject(speed);
            System.out.println("Running eject at " + speed);
        }).withName("Eject").finallyDo((interrupted) -> {
            stopIntakeMotors();
            System.out.println("Eject stopped. Interrupted: " + interrupted);
        });
    }

    @Override
    public void periodic() {
        // Log motor outputs
        Logger.recordOutput("AlgaeArm/LeftIntakeMotorOutput", leftIntakeMotor.getMotorOutputPercent());
        Logger.recordOutput("AlgaeArm/RightIntakeMotorOutput", rightIntakeMotor.getMotorOutputPercent());
        // We would log the lift motor position here if possible? Something like below?
        // Logger.recordOutput("AlgaeArm/LiftMotorPosition", liftMotor.getSelectedSensorPosition()); 
    }
}