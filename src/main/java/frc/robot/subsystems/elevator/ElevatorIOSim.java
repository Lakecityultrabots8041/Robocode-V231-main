package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getFalcon500(2),  // get the Falcon 500 gearbox and number of motors
        4.0,               // gear ratio (adjust here if you change the gearbox)
        5.0,         // carriage mass in kg
        0.05,     // pulley radius in meters
        0.0,      // minimum height
        1.32,     // maximum height (52 inches)
        true,     // simulate gravity is reqiured so it can apply torgue to the motor
        0.02 // measurement noise standard deviation (tune as needed)
    );
    private final PIDController controller = new PIDController(0.1, 0.0, 0.0);
    private final ElevatorFeedforward ff = new ElevatorFeedforward(0.2, 0.05, 0.0);
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);  // Simulate a 20ms loop

        inputs.position = sim.getPositionMeters();
        inputs.velocity = sim.getVelocityMetersPerSecond();
        inputs.appliedVoltsLeader = appliedVolts;
        inputs.appliedVoltsFollower = appliedVolts;
        inputs.supplyCurrentLeader = sim.getCurrentDrawAmps();
        inputs.supplyCurrentFollower = sim.getCurrentDrawAmps();
        inputs.torqueCurrentLeader = sim.getCurrentDrawAmps();
        inputs.torqueCurrentFollower = sim.getCurrentDrawAmps();
        inputs.temperatureLeader = 0.0;
        inputs.temperatureFollower = 0.0;
        inputs.setpointPosition = controller.getSetpoint();
        inputs.setpointVelocity = 0.0;  // No velocity setpoint in this example
    }

    @Override
    public void runSetpoint(double targetPositionMeters) {
        double currentHeight = sim.getPositionMeters();
        double currentVelocity = sim.getVelocityMetersPerSecond();

        double controllerVoltage = controller.calculate(currentHeight, targetPositionMeters);
        double feedForwardVoltage = ff.calculate(currentVelocity);

        double effort = MathUtil.clamp(controllerVoltage + feedForwardVoltage, -12, 12);
        runVolts(effort);
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }
}
