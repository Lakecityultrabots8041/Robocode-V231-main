package frc.robot.constants;


/**
 * Constants for the Elevator subsystem. This class should contain all the
 * constant values for the Elevator subsystem, including motor IDs, PID
 * constants, and any other constants that are needed.
 */

public final class Elevator_Constants {
    // PID
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.005;
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kD = 0.1;
    

    //motition magic constants
    public static final int CRUISE_VELOCITY = 80;
    public static final int  ACCELERATION = 160;
    
    
    // Motor IDs
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 6;


    //-------------------------Elevator Height Presets-------------------------//

    // Adjust these values based on robot-field dimensions(in meters) //
   
    public static final double Home_Position = 1.0;        // returns elevator and arm to park position
    public static final double Player_Coral_Load_Height = 45.2;// Human player station height

    
    public static final double L1_Bottom_Score = -5.0;       // Bottom scoring
    public static final double L2_Middle_Score = 37.9;       // Middle scoring
    public static final double L3_TOP_Score = 99.0;       // Top scoring
    public static final double Intake_LV = 54.0;         // Coral intake level

    public static final double Processor_Position = 2.5;   // Safe carrying position with algearm to move to Processor locations


    // Adjust these values based on your robot's physical constraints
    public static final double MAX_HEIGHT = 55.0;    //Prevent elevator from going above this height
    public static final double MIN_HEIGHT = 1.0;   //Prevent elevator from going below this height
    public static final double MAX_CURRENT = 40.0;          // Maximum current draw


    private Elevator_Constants() {}
}