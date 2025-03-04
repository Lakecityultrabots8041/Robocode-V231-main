package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Elevator_Constants;
import org.littletonrobotics.junction.Logger;


public class Elevator_Subsystem extends SubsystemBase {

 /*    public enum ElevatorPosition {
        Home,
        L1,
        L2,
        L3,
        Processor,
        Station;
    }

    public enum ElevatorTarget {
        Home(Elevator_Constants.Home_Position),
        L1(Elevator_Constants.L1_Bottom_Score),
        L2(Elevator_Constants.L2_Middle_Score),
        L3(Elevator_Constants.L3_TOP_Score),
        Processor(Elevator_Constants.Processor_Position),
        Station(Elevator_Constants.Station_LV);
    }
*/

// ---------- Configure the motors for the elevator subsystem --------------------
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

//--------------------Control Modes------------------------------------------------------
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage manualMoveControl = new VelocityVoltage(0);
    //private final NeutralOut stopControl = new NeutralOut();
    
    public Elevator_Subsystem() {
     
    // Initialize the elevator subsystem
    leftMotor = new TalonFX(Elevator_Constants.LEFT_MOTOR_ID, "rio");
    rightMotor = new TalonFX(Elevator_Constants.RIGHT_MOTOR_ID, "rio");
    configureMotors();
  }

    // Create the control modes for the elevator with set target height

   /* public void setTargetHeight(double targetPositionMeters) {
       final MotionMagicExpoVoltage mmReq = new MotionMagicExpoVoltage(0);
       leftMotor.setControl(mmReq.withPosition(targetPositionMeters));
   }*/

    
   
    private void configureMotors() {
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        // Define Feedback Sensors
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.SensorToMechanismRatio = 1.0; 
        leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // Slot0 PID configuration
        leftMotorConfig.Slot0.kS = Elevator_Constants.kS;
        leftMotorConfig.Slot0.kV = Elevator_Constants.kV;
        leftMotorConfig.Slot0.kA = Elevator_Constants.kA;
        leftMotorConfig.Slot0.kP = Elevator_Constants.kP;
        leftMotorConfig.Slot0.kI = Elevator_Constants.kI;
        leftMotorConfig.Slot0.kD = Elevator_Constants.kD;
        
      
        //--------------------Motion Magic Configurations--------------------

        // Motion Magic Cruise Velocity and Acceleration
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator_Constants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = Elevator_Constants.ACCELERATION;

        
        // Apply's brake mode to motors when not moving
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            

        // Apply new configuration to motors from the configure motors 
        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    
        // Invert right motor and set to follow left motor
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
        
        resetEncoders();
    }


    // ---- ADDED FOR MANUAL CONTROL ----
    public void moveUp(double speed) {
        //We need to check the max height our elevator can go to and set that to constant MAX_HEIGHT
        if (getCurrentHeight() < Elevator_Constants.MAX_HEIGHT) {
            //double velocity = Elevator_Constants.CRUISE_VELOCITY * speed;
           // leftMotor.setControl(manualMoveControl.withVelocity(velocity));
           leftMotor.set(0.40);
        } else {
            stopMotion();
        }
    }

    public void moveDown(double speed) {
        // Please for all that is holy don't let the elevator go below the minimum height
        if (getCurrentHeight() > Elevator_Constants.MIN_HEIGHT) {
            //double velocity = -Elevator_Constants.CRUISE_VELOCITY * speed;  //apply (negative) constants to move down
            //leftMotor.setControl(manualMoveControl.withVelocity(velocity));
            leftMotor.set(-0.40);
        } else {
            stopMotion();
        }
    }

    public void moveUpFast(double speed) {
        //We need to check the max height our elevator can go to and set that to constant MAX_HEIGHT
        if (getCurrentHeight() < Elevator_Constants.MAX_HEIGHT) {
            //double velocity = Elevator_Constants.CRUISE_VELOCITY * speed;
           // leftMotor.setControl(manualMoveControl.withVelocity(velocity));
           leftMotor.set(0.80);
        } else {
            stopMotion();
        }
    }

    public void moveDownFast(double speed) {
        // Please for all that is holy don't let the elevator go below the minimum height
        if (getCurrentHeight() > Elevator_Constants.MIN_HEIGHT) {
            //double velocity = -Elevator_Constants.CRUISE_VELOCITY * speed;  //apply (negative) constants to move down
            //leftMotor.setControl(manualMoveControl.withVelocity(velocity));
            leftMotor.set(-0.80);
        } else {
            stopMotion();
        }
    }

    public void stopMotion() {
        leftMotor.stopMotor();
    }


    public void setTargetPosition(double targetPositionRotations) {
        double clampedPosition = Math.min(Math.max(targetPositionRotations, Elevator_Constants.MIN_HEIGHT), Elevator_Constants.MAX_HEIGHT);
        leftMotor.setControl(motionMagicRequest.withPosition(clampedPosition));
    }

   
    public double getCurrentHeight() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public void moveToPreset(double presetPosition) {
        setTargetPosition(presetPosition);
    }

    public boolean isAtPosition(double targetPosition) {
        double currentPosition = getCurrentHeight();
        return Math.abs(currentPosition - targetPosition) < Elevator_Constants.POSITION_TOLERANCE;
    }

    public void resetEncoders() {
        leftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // Log data for debugging and dashboards
       // Logger.recordOutput("Elevator/Position", getCurrentHeight());
       // Logger.recordOutput("Elevator/Velocity", leftMotor.getVelocity().getValueAsDouble());
       // Logger.recordOutput("Elevator/LeftMotorCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
        //Logger.recordOutput("Elevator/RightMotorCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
        
        // Call the visualizer to update the height of the elevator periodically
        //visualizer.update(edu.wpi.first.units.Units.Meters.of(getCurrentHeight()));
    }
    
}

