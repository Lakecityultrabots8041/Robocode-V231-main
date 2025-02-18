

// -------------------------------------------------------------------------- ONLY USED FOR SIMULATION IF YOU ARE NOT SIMULATING DO NOT TOUCH ------------------------------------------------------------------------------------------

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.IOInputs;

public class ElevatorIOSim implements ElevatorIO {
    // Match the real elevator's conversion factor
    private static final double ROTATIONS_TO_METERS = Math.PI * 0.0254 * 1.751; // Same as real implementation

    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getFalcon500(2),    // 2 Falcon 500s
        4.0,                        // Gear ratio
        5.0,                        // Carriage mass in kg
        Units.inchesToMeters(1.751), // Pulley radius (matching real implementation)
        0.0,                        // Min height
        1.32,                       // Max height
        true,                       // Simulate gravity
        0.02                        // Measurement noise
    );

    // Motion Magic simulation components
    private double targetPosition = 0.0;
    private double currentVelocity = 0.0;
    private double currentPosition = 0.0;
    private double appliedVolts = 0.0;
    private final double maxVelocity = ElevatorConstants.CRUISE_VELOCITY;
    private final double maxAcceleration = ElevatorConstants.ACCELERATION;

    // Create a feedforward controller to simulate the Motion Magic behavior
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kF,
        0.0  // No acceleration feedforward for now
    );

    // PID controller with the same gains as the real implementation
    private final PIDController pidController = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD
    );

    @Override
    public void updateInputs(IOInputs inputs) {
        // Calculate motion profile
        double positionError = targetPosition - currentPosition;
        double desiredVelocity = 0.0;

        // Simple trapezoidal motion profile
        if (Math.abs(positionError) > 0.001) {
            // Determine desired velocity based on position error
            desiredVelocity = Math.signum(positionError) * maxVelocity;
            
            // Limit acceleration
            double velocityError = desiredVelocity - currentVelocity;
            double allowedVelocityChange = maxAcceleration * 0.02; // 20ms loop time
            currentVelocity += MathUtil.clamp(velocityError, -allowedVelocityChange, allowedVelocityChange);
        }

        // Calculate voltage using PID and feedforward
        double pidOutput = pidController.calculate(currentPosition, targetPosition);
        double ffOutput = feedforward.calculate(currentVelocity);
        double voltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        // Update simulation
        sim.setInputVoltage(voltage);
        sim.update(0.02);  // 20ms update rate

        // Update current position from simulation
        currentPosition = sim.getPositionMeters();
        
        // Update inputs structure
        inputs.position = currentPosition / ROTATIONS_TO_METERS; // Convert to rotations
        inputs.velocity = sim.getVelocityMetersPerSecond() / ROTATIONS_TO_METERS;
        inputs.appliedVoltsLeader = voltage;
        inputs.appliedVoltsFollower = voltage;
        inputs.supplyCurrentLeader = sim.getCurrentDrawAmps();
        inputs.supplyCurrentFollower = sim.getCurrentDrawAmps();
    }

    @Override
    public void runSetpoint(double targetPositionMeters) {
        // Convert from meters to our internal units
        this.targetPosition = targetPositionMeters;
        pidController.setSetpoint(targetPositionMeters);
    }

    @Override
    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }

    @Override
    public void stop() {
        targetPosition = currentPosition; // Stop at current position
        currentVelocity = 0.0;
        sim.setInputVoltage(0.0);
    }

    @Override
    public void runVolts(double volts) {
        sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }
}