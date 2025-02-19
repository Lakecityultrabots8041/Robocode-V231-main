package frc.robot.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * Constants for the AlgaeArm subsystem. This class should contain all the
 * constant values for the AlgaeArm subsystem, including motor IDs, PID
 * constants, and any other constants that are needed.
 */


public class AlgaeArm_Constants {
    public static final double kS = .25;
    public static final double kV = .12;
    public static final double kA = 0.01;
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kd = 0.1;

    // Optional constant for gravity type (if you need to set it on the config)
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
    
    // Motor IDs
     public static final int MOTOR_ID = 0;

     // Set other motors here
     // public static final int OTHER_MOTOR_ID = 1;
     // public static final int OTHER_MOTOR_ID = 2; 
    
    //motition magic constants
    public static final int CRUISE_VELOCITY = 15;
    public static final int ACCELERATION = 20;
    public static final int JERK = 30;

    // **Preset arm positions (units must match your sensor's output)**
    public static final double ARM_LOWER_POSITION = 0.449707;
    public static final double ARM_UPPER_POSITION = 1.594238;
    
    

     // Add any preset positions or tolerances here as needed
    private AlgaeArm_Constants() {} // Prevent instantiation

    
}
