package frc.robot.subsystems.elevator;

public interface ElevatorIOs {

    /**
     * Updates the current elevator inputs (position, velocity, voltage, etc.).
     */
    void updateInputs(ElevatorIOInputs inputs);

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