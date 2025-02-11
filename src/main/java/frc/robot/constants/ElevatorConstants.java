package frc.robot.constants;

public final class ElevatorConstants {
    public static final double kF = 0.05;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    //motition magic constants
    public static final int CRUISE_VELOCITY = 15000;
    public static final int ACCELERATION = 6000;

    // Motor IDs
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 6;

    // Preset positions (in meters)
    public static final double LEVEL_1 = 0.0;
    public static final double LEVEL_2 = 0.75;
    public static final double LEVEL_3 = 1.5;
    public static final double LEVEL_4 = 2.25;
    
    // Add any preset positions or tolerances here as needed
    private ElevatorConstants() {} // Prevent instantiation
}