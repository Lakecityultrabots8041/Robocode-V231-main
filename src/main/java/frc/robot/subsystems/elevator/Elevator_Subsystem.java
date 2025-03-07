package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;


public class Elevator_Subsystem extends SubsystemBase {

    
// ---------- Configure the motors for the elevator subsystem --------------------
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;


    private double targetPosition = ElevatorConstants.HOME_POSITION;

    //--------------------Control Modes------------------------------------------------------
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0); // Used for precise position control with motion profile. Uses acceleration and deacceleration for target position. 
    private final VelocityVoltage manualMoveControl = new VelocityVoltage(0); // Used for manual control for up and down where you want to move a constant speed rather then position target. 


    public Elevator_Subsystem() {
     
    //Define Motors here\\
    leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "rio");
    rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "rio");

    configureElevTalons(); // Configure the talons for the elevator


    // Set the start position to the home position
    targetPosition = ElevatorConstants.HOME_POSITION;

    //Put the elevator on the dashboard
    SmartDashboard.putData("Elevator", this);



  }
    
   // Now lets configure the actual motors and do some neat things

    private void configureElevTalons() {
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        //Configure PIDs for motors
        leftMotorConfig.Slot0.kP = ElevatorConstants.kP;
        leftMotorConfig.Slot0.kI = ElevatorConstants.kI;
        leftMotorConfig.Slot0.kD = ElevatorConstants.kD;
        leftMotorConfig.Slot0.kS = ElevatorConstants.kS;
        leftMotorConfig.Slot0.kV = ElevatorConstants.kV;
        leftMotorConfig.Slot0.kA = ElevatorConstants.kA;
        leftMotorConfig.Slot0.kG = ElevatorConstants.kG;

        //Configure Feedback Sensor inside motors
        leftMotorConfig.Feedback.SensorToMechanismRatio = 40.0; // 40:1 gear ratio
        leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // Use the encoder on the motor

        // Motion Magic Cruise Velocity and Acceleration
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        // SAFETY FIRST for elevator heights
        leftMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leftMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT;
        leftMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leftMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_HEIGHT;

        // Apply's brake mode to motors when not moving
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply Limit for motor protection
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.MAX_CURRENT;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            

        // Apply new configuration to motors from the configure motors 
        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    
        // Invert right motor and set to follow left motor
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(0.0); // is this needed?
        Logger.recordOutput("Elevator/MotorConfigured", true);
    }


    /**
     * Sets the target position for the elevator.
     * 
     * @param position The target position in encoder units
     */
    public void setTargetPosition(double position) {
        // Clamp position to valid range
        position = Math.min(Math.max(position, ElevatorConstants.MIN_HEIGHT), ElevatorConstants.MAX_HEIGHT);
        
        // Update target position
        targetPosition = position;
        
        // Apply motion magic control
        leftMotor.setControl(motionMagicRequest.withPosition(position));
    }
    
    /**
     * Gets the current target position.
     * 
     * @return The current target position
     */
    public double getTargetPosition() {
        return targetPosition;
    }
    
    /**
     * Gets the current actual position.
     * 
     * @return The current encoder position
     */
    public double getCurrentPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Checks if the elevator is at the target position.
     * 
     * @return true if the elevator is at the target position
     */
    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < ElevatorConstants.POSITION_TOLERANCE;
    }
    
    /**
     * Moves the elevator up at the specified speed.
     * 
     * @param speed The speed to move up at (0 to 1)
     */
    public void moveUp(double speed) {
        // Ensure we don't exceed max height
        if (getCurrentPosition() >= ElevatorConstants.MAX_HEIGHT) {
            stopMotion();
            return;
        }
                
        // Use velocity control for smooth manual movement
        double velocityRotationsPerSecond = speed * 60; // Convert to rotations per second
        leftMotor.setControl(manualMoveControl.withVelocity(velocityRotationsPerSecond));
    }
    
    /**
     * Moves the elevator down at the specified speed.
     * 
     * @param speed The speed to move down at (0 to 1)
     */
    public void moveDown(double speed) {
        // Ensure we don't go below min height
        if (getCurrentPosition() <= ElevatorConstants.MIN_HEIGHT) {
            stopMotion();
            return;
        }
        
        // Use velocity control for smooth manual movement
        double velocityRotationsPerSecond = -speed * 60; // Negative for down motion
        leftMotor.setControl(manualMoveControl.withVelocity(velocityRotationsPerSecond));
 }
    
    /**
     * Stops all elevator motion. This is used in case holdPosition doesn't work. 
     */
    public void stopMotion() {
        leftMotor.stopMotor();
    }
    
    /**
     * Holds the elevator at its current position.
     */
    public void holdPosition() {
        setTargetPosition(getCurrentPosition());
    }
    
@Override
public void periodic() {
    // Update SmartDashboard with current position and target
    SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
    SmartDashboard.putNumber("Elevator Target", targetPosition);
    SmartDashboard.putBoolean("Elevator At Target", isAtTargetPosition());
}
 
public double getCurrentHeight() {
    return leftMotor.getPosition().getValueAsDouble();
}

public void moveToPreset(double presetPosition) {
    setTargetPosition(presetPosition);
}

public boolean isAtPosition(double targetPosition) {
    double currentPosition = getCurrentHeight();
    return Math.abs(currentPosition - targetPosition) < ElevatorConstants.POSITION_TOLERANCE; // uses math.abs to get the absolute value of the difference between the current position and the target position
}

public void resetEncoders() {
    leftMotor.setPosition(0.0); // Changed to setPosition instead of set(speed)
}
    
//--------------------------------------------------------------------------
    // Command Getters
    //--------------------------------------------------------------------------
    
    /**
     * Gets a command that moves the elevator up at the default speed.
     * 
     * @return A command to move the elevator up
     */
    public Command getManualUpCommand() {
        return Commands.run(() -> moveUp(ElevatorConstants.MANUAL_UP_SPEED), this)
            .finallyDo((interrupted) -> holdPosition())// When button assigned is release, it calls this finallyDo to stop the motor
            .withName("Elev Manual Up"); // this then puts the command on the dashboard for logging purposes
    }
    
    /**
     * Gets a command that moves the elevator down at the default speed.
     * 
     * @return A command to move the elevator down
     */
    public Command getManualDownCommand() {
        return Commands.run(() -> moveDown(ElevatorConstants.MANUAL_DOWN_SPEED), this)
            .finallyDo((interrupted) -> holdPosition())
            .withName("Elev Manual Down");
    }


    public Command getManualUpFastCommand() {
        return Commands.run(() -> moveUp(ElevatorConstants.MANUAL_UP_FAST_SPEED), this)
            .finallyDo((interrupted) -> holdPosition())
            .withName("Elev Manual Up Fast");
    }


    public Command getManualDownFastCommand() {
        return Commands.run(() -> moveUp(ElevatorConstants.MANUAL_DOWN_FAST_SPEED), this)
            .finallyDo((interrupted) -> holdPosition())
            .withName("Elev Manual Up Fast");
    }

    /**
     * Gets a command that holds the elevator at its current position.
     * 
     * @return A command to hold the elevator position
     */
    public Command getHoldCommand() {
        return Commands.run(() -> holdPosition(), this)
            .withName("Elev Hold");
    }
    
    /**
     * Gets a command that moves the elevator to the home position.
     * 
     * @return A command to move the elevator to home
     */
    public Command getHomeCommand() {
        return Commands.runOnce(() -> setTargetPosition(ElevatorConstants.HOME_POSITION), this)
            .withName("Elev Home");
    }
    
    /**
     * Gets a command that moves the elevator to a specific preset position.
     * 
     * @param position The preset position from ElevatorConstants.Position
     * @return A command to move the elevator to the preset position
     */
    public Command getPresetPositionCommand(ElevatorConstants.Position position) {
        return Commands.runOnce(() -> setTargetPosition(position.getValue()), this)
            .withName("Elev to " + position.name());
    }
}

