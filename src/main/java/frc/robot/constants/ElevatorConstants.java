package frc.robot.constants;

public final class ElevatorConstants {
    // PID
    public static final double kP = 6.3;
    public static final double kI = 3.7;
    public static final double kD = 0.03;
    public static final double kF = 0.05;  // Feed forward
    public static final double kS = 0.2; // Static friction compensation
    public static final double kG = 0.41;   // Gravity compensation if needed
    //motition magic constants
    public static final int CRUISE_VELOCITY = 30;
    public static final int  ACCELERATION = 16;
    
    
    // Motor IDs
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 6;

    // Motor current limit
    public static final double ElevatorMotorCurrentLimit = 30; // Amps




    //-------------------------Elevator Height Presets-------------------------//

    // Adjust these values based on robot-field dimensions(in meters) //
   
    public static final double STOWED_HEIGHT = 0.0;          // Bottom position
    public static final double CORAL_PICKUP_HEIGHT = 0.3;    // Height for coral intake
    public static final double FEEDER_HEIGHT = 1.2;         // Human player station height
    
    public static final double ALGAE_L1_HEIGHT = 0.4;       // Bottom scoring
    public static final double ALGAE_L2_HEIGHT = 0.8;       // Middle scoring
    public static final double ALGAE_L3_HEIGHT = 1.5;       // Top scoring
    public static final double NET_SCORE_HEIGHT = 1.8;      // Net scoring position
    
    public static final double PROCESSOR_HEIGHT = 0.6;      // Transfer position
    public static final double CARRY_HEIGHT = 0.5;          // Safe carrying height

    // Safety Limits
    // Adjust these values based on your robot's physical constraints
    public static final double MAX_HEIGHT = 2.0;            // Maximum safe height
    public static final double MIN_HEIGHT = -0.1;           // Minimum height (for zeroing) // Idea for negative zeroing helps get us closer to 0
    public static final double MAX_CURRENT = 40.0;          // Maximum current draw

   // OTHER PID TWEAKS
/*  public static final double kIZone = 0.0; // Integral zone
    public static final double kPeakOutput = 1.0; // Peak output for the motor
    public static final double kMinOutput = -1.0; // Minimum output for the motor
    public static final double kMaxOutput = 1.0; // Maximum output for the motor
    public static final double kMaxIntegralAccumulator = 1.0; // Maximum integral accumulator value
    public static final double kMaxError = 0.1; // Maximum error for the PID controller
    public static final double kMaxVelocity = 1.0; // Maximum velocity for the elevator
    public static final double kMaxAcceleration = 1.0; // Maximum acceleration for the elevator
    public static final double kMaxJerk = 1.0; // Maximum jerk for the elevator
    public static final double kMaxPosition = 1.0; // Maximum position for the elevator
    public static final double kMinPosition = 0.0; // Minimum position for the elevator
    */

//-------------------------IO Inputs---------------------------------------------------------------------------------------------------------------------//
            // This class is used to store the inputs from the elevator subsystem and update them in the IO layer
    public static class IOInputs {
        public double position = 0.0;  // Position in meters
        public double velocity = 0.0;  // Velocity in meters per second
        public double appliedVoltsLeader = 0.0;
        public double appliedVoltsFollower = 0.0;
        public double supplyCurrentLeader = 0.0;
        public double supplyCurrentFollower = 0.0;
        public double torqueCurrentLeader = 0.0;
        public double torqueCurrentFollower = 0.0;
        public double temperatureLeader = 0.0;
        public double temperatureFollower = 0.0;
        public double setpointPosition = 0.0;
        public double setpointVelocity = 0.0;
    }
    
  

    // Preset positions (in meters)
    // Adjust these values based on your elevator's physical setup with tuner x
    public static final double LEVEL_1 = 0.0;
    public static final double LEVEL_2 = 0.75;
    public static final double LEVEL_3 = 1.5;
    public static final double LEVEL_4 = 2.25;
    
    // Add any preset positions or tolerances here as needed
    private ElevatorConstants() {} // {} <---- Prevents instantiation when bot boots up
}