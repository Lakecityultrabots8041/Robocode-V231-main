package frc.robot.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * Constants for the AlgaeArm subsystem. This class should contain all the
 * constant values for the AlgaeArm subsystem, including motor IDs, PID
 * constants, and any other constants that are needed.
 */


public class AlgaeArm_Constants {
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kd = 0.1;
    public static final double kS = 2.0;
    public static final double kV = 4.5;
    public static final double kA = 0.01;
    public static final double kG = 4.25;
  
  

    // Optional constant for gravity type (if you need to set it on the config)
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
    
    // Motor IDs
     public static final int MOTOR_ID = 0;


    //motition magic constants
    public static final int CRUISE_VELOCITY = 15;
    public static final int ACCELERATION = 20;
    public static final int JERK = 20;

    // **Preset arm positions (units must match your sensor's output)**
    public static final double ARM_LOWER_POSITION = -0.512;
    public static final double ARM_UPPER_POSITION = 2.703;

    // arm position for L3 to reach higher algae
    public static final double ARM_UPPER_POSITION_L3 = 3.119;
    //possibly make higher algae position a preset position*********

     // Add any preset positions or tolerances here as needed
    private AlgaeArm_Constants() {} // Prevent instantiation

    
}
