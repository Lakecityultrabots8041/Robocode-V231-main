package frc.robot.constants;


/**
 * Constants for the Elevator subsystem. This class should contain all the
 * constant values for the Elevator subsystem, including motor IDs, PID
 * constants, and any other constants that are needed.
 */

public final class Elevator_Constants {
    // PID
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.005;
    public static final double kG = 0.35; //tune this in phx tuner
    

    //motition magic constants
    public static final int CRUISE_VELOCITY = 80;
    public static final int  ACCELERATION = 160;

    //Acceptable error
    public static final double POSITION_TOLERANCE = 1.0;
    
    // Motor IDs
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 6;


    //-------------------------Elevator Height Presets-------------------------//

    // Adjust these values based on robot-field dimensions(in meters) //
   
    public static final double Home_Position = 30.9750;        // returns elevator and arm to park position
    
    public static final double L1_Bottom_Score = 40.0;       // Bottom scoring
    public static final double L2_Middle_Score = 37.9;       // Middle scoring
    public static final double L3_TOP_Score = 105.0;       // Top scoring
    public static final double Station_LV = 50.0;         // Coral intake level 
    // lets rename the above to something that states player or human involved

    public static final double Processor_Position = 10.0;   // Safe carrying position with algearm to move to Processor locations


    // Adjust these values based on your robot's physical constraints
    public static final double MAX_HEIGHT = 111.0;    //Prevent elevator from going above this height
    public static final double MIN_HEIGHT = 5.0;   //Prevent elevator from going below this height
    public static final double MAX_CURRENT = 40.0;          // Maximum current draw

    // control for manual elevator

    public static final double MANUAL_UP_SPEED = 0.5; // speed for manual control
    public static final double MANUAL_DOWN_SPEED = -0.5; // speed for manual control


    //make a gear ratio for elevator motors

    public static final double GEAR_RATIO = 40.0; // 40:1 gear ratio

    // Timeouts
    public static final double TIMEOUT = 5.0; // 5 seconds


    private Elevator_Constants() {}
}