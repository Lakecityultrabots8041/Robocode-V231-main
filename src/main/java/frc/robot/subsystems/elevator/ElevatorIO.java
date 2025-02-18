package frc.robot.subsystems.elevator;
import frc.robot.constants.ElevatorConstants.IOInputs;

public interface ElevatorIO {

    

    /**
     * Updates the current elevator inputs (position, velocity, voltage, etc.).
     */
    void updateInputs(IOInputs inputs);

    /**
     * Runs the elevator to a specific setpoint position (in meters)
     */
    void runSetpoint(double targetPositionMeters);

    /**
     * Runs the elevator using the specified voltage.
     */
    void runVolts(double volts);

    /**
     * Sets the PID gains dynamically.
     */
    void setPID(double p, double i, double d);

    /**
     * Stops the elevator.
     */
    void stop();
}