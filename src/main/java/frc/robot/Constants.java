package frc.robot;
import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * The Constants class provides a centralized location for all robot-wide constant values.
 * This class should not be constructed - it is simply a holder for static constant values
 */
public final class Constants {
    public static final boolean REAL = true; // Only needed if we have some detection for simulation vs real, ie REAL is always true since we assume the bot will be running this code. 

    /* /---------------------------------------------\
     * /-----ELEVATOR CONSTANTS-----------------------\
     * /-----------------------------------------------\*/

    public static final class ElevatorConstants {

        // Motor IDs
        public static final int LEFT_MOTOR_ID = 5;
        public static final int RIGHT_MOTOR_ID = 6;
        
        // PID Constants
        public static final double kP = 4.8;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.005;
        public static final double kG = 0.35;
        
        // Motion Magic Constants
        public static final int CRUISE_VELOCITY = 80;
        public static final int ACCELERATION = 160;
        
        // Position Limits
        public static final double MIN_HEIGHT = 5.0;      // Minimum safe position
        public static final double MAX_HEIGHT = 111.0;    // Maximum safe position
        
        // Preset positions
        public static final double HOME_POSITION = 30.975;
        public static final double L1_BOTTOM_SCORE = 40.0;    // Bottom scoring
        public static final double L2_MIDDLE_SCORE = 37.9;    // Middle scoring
        public static final double L3_TOP_SCORE = 105.0;      // Top scoring
        public static final double STATION_LV = 50.0;         // Coral intake level
        public static final double PROCESSOR_POSITION = 10.0; // Safe carrying position
        
        // Tolerances and limits
        public static final double POSITION_TOLERANCE = 0.1;   // Tolerance for considering position reached
        public static final double MAX_CURRENT = 40.0;         // Maximum current draw
        
        // Manual control speeds
        public static final double MANUAL_UP_SPEED = 0.5;      // Speed for manual control
        public static final double MANUAL_DOWN_SPEED = 0.5;    // Speed for manual control
        
        // Other constants
        public static final double GEAR_RATIO = 40.0;          // 40:1 gear ratio
        public static final double TIMEOUT = 5.0;              // Timeout in seconds
        
        /**
         * Enum representing preset elevator positions.
         * This provides a type-safe way to specify elevator positions in code.
         */
        public enum Position {
            HOME(HOME_POSITION),
            L1(L1_BOTTOM_SCORE),
            L2(L2_MIDDLE_SCORE),
            L3(L3_TOP_SCORE),
            STATION(STATION_LV),
            PROCESSOR(PROCESSOR_POSITION);
            
            private final double value;
            
            Position(double value) {
                this.value = value;
            }
            
            public double getValue() {
                return value;
            }
        }
    }
    
    /**
     * Constants for the Algae Arm subsystem.
     */
    public static final class AlgaeArmConstants {

        // Only needed if we have some detection for simulation vs real, ie REAL is always true since we assume the bot will be running this code. 
        public static final boolean REAL = true;
        
        // Motor IDs 
        public static final int MOTOR_ID = 0;

        // PID Constants 
        public static final double kP = 30;
        public static final double kI = 0;
        public static final double kd = 0.1;
        public static final double kS = 2.0;
        public static final double kV = 4.5;
        public static final double kA = 0.01;
        public static final double kG = 4.25;
        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
        
        //motition magic constants
        public static final int CRUISE_VELOCITY = 15;
        public static final int ACCELERATION = 20;
        public static final int JERK = 20;

        // **Preset arm positions (units must match your sensor's output)**
        public static final double ARM_STOWED_POSITION = 0.1; // figure out safe position for stowing arm down when not needed

        public static final double ARM_LOWER_POSITION = -0.407715; // Get Values for arm at 
        public static final double ARM_UPPER_POSITION = 2.703; // **Put in new values for this**
        

        // Enum for preset positions
        public enum Position {
            STOWED(ARM_STOWED_POSITION),
            ARM_LOW_SCORE(ARM_LOWER_POSITION),
            ARM_HIGH_SCORE(ARM_UPPER_POSITION);
        
            private final double value;
            
            Position(double value) {
                this.value = value;
            }
            
            public double getValue() {
                return value;
            }
        }
    }
    
    /**
     * Constants for the Coral subsystem.
     * 
     */
    public static final class CoralConstants {
        public static final boolean REAL = true; // Only needed if we have some detection for simulation vs real, ie REAL is always true since we assume the bot will be running this code. 
        
        // Motor IDs 
        public static final int MOTOR_ID = 7;
        
        // Speed constants
        public static final double INTAKE_SPEED = 0.7;
        public static final double OUTTAKE_SPEED = -0.5;
    }
    
    /**
     * Constants for automatic commands and sequences.
     * These constants apply to multiple subsystems.
     */



     // Ill have to think what i can use AutoConstants for
    /* 
    public static final class AutoConstants {
        public static final double AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS = 2.0;
        public static final double TIME_UNTIL_CORAL_IS_SCORED_SECONDS = 0.25;
    }
    */
    private Constants() {
        // Private constructor to prevent instantiation
    }
}