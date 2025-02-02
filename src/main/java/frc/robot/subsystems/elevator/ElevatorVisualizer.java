package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;



import static edu.wpi.first.units.Units.*;

public class ElevatorVisualizer {
    private final String key;
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevator;

    // Adjustable angle modifiers for 3D visualization
    private final double elevatorXModifier;
    private final double elevatorZModifier;

    public ElevatorVisualizer(String key, Color color) {
        this.key = key;

        // Calculate 45-degree projection modifiers
        this.elevatorXModifier = -Math.cos(Degrees.of(45).in(Radians));
        this.elevatorZModifier = Math.sin(Degrees.of(45).in(Radians));

        // Initialize the 2D mechanism visualization
        this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kWhite));
        this.root = panel.getRoot("elevator", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
        this.elevator = root.append(
            new MechanismLigament2d(
                "Elevator",
                Inches.of(0).in(Meters),
                90,  // Angle for a vertical elevator
                10,
                new Color8Bit(color)
            )
        );

        // Display the mechanism on SmartDashboard
        SmartDashboard.putData("Elevator Visualization (" + key + ")", this.panel);
    }

    public void update(Distance position) {
        // Update 2D visualization with the current position
        elevator.setLength(position.in(Meters));
    
        // Calculate 3D projection for AdvantageKit logging
        Distance elevatorX = position.times(elevatorXModifier);
        Distance elevatorZ = position.times(elevatorZModifier);
        double elevatorXMeters = elevatorX.in(Meters);  
        double elevatorZMeters = elevatorZ.in(Meters);
    
        Pose3d elevatorPose = new Pose3d(
            elevatorXMeters,  // X-coordinate
            0.0,                 // Y-coordinate (assuming elevator is linear along the X-Z plane)
            elevatorZMeters,  // Z-coordinate
            new Rotation3d()       // Default rotation (modify if needed)
        );
    
        // Log 3D pose for visualization in AdvantageScope using the appropriate logger.
        Logger.recordOutput("Elevator/Mechanism3d/" + key, elevatorPose);
        
        // If you need to log the numerical data separately with another logger, do it here.
        // For example:
        // Logger.getCurrentLogger().recordOutput("Elevator/Mechanism3d/" + key, elevatorPose);
    }
}