package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;


public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx, ty, tv, tid;

    private final PIDController alignController;
    
    
    private static final double ALIGN_KP = 0.04;
    private static final double ALIGN_KI = 0.0;
    private static final double ALIGN_KD = 0.005;
    private static final double ALIGN_TOLERANCE = 2.0; // degrees
    private static final double MAX_ALIGN_SPEED = 0.4; // 40% max rotation
    
    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        tv = limelightTable.getEntry("tv");
        tid = limelightTable.getEntry("tid");
        
        // Configure PID
        alignController = new PIDController(ALIGN_KP, ALIGN_KI, ALIGN_KD);
        alignController.setSetpoint(0.0); // Center target
        alignController.setTolerance(ALIGN_TOLERANCE);
        
        // Set Limelight to vision mode with pipeline control
        limelightTable.getEntry("ledMode").setNumber(0);
        limelightTable.getEntry("camMode").setNumber(0);
    }
    
    @Override
    public void periodic() {
        // Read from Limelight
        boolean hasTarget = tv.getDouble(0) == 1;
        double targetX = tx.getDouble(0.0);
        double targetY = ty.getDouble(0.0);
        int targetID = (int) tid.getInteger(-1);
        
        // Update RobotState
        RobotState.getInstance().updateVisionTarget(hasTarget, targetX, targetY, targetID);
        RobotState.getInstance().setAlignedToTarget(
            hasTarget && alignController.atSetpoint()
        );
    }
    
    /**
     * STATELESS: Get rotation correction to align with target
     * Returns 0 if no target visible
     * 
     * @return rotation speed (-1.0 to 1.0)
     */
    public double getAlignmentRotation() {
        if (!RobotState.getInstance().hasVisionTarget()) {
            alignController.reset();
            return 0.0;
        }
        
        double output = -alignController.calculate(RobotState.getInstance().getVisionTargetX());
        return Math.max(-MAX_ALIGN_SPEED, Math.min(MAX_ALIGN_SPEED, output));
    }
    
    /**
     * STATELESS: Get rotation correction only if target within threshold
     * 
     * @param maxAngle Maximum angle to attempt alignment (degrees)
     * @return rotation speed, or 0 if target too far
     */
    public double getAlignmentRotation(double maxAngle) {
        if (!RobotState.getInstance().hasVisionTarget() || 
            Math.abs(RobotState.getInstance().getVisionTargetX()) > maxAngle) {
            return 0.0;
        }
        return getAlignmentRotation();
    }
    
    /**
     * Reset PID controller
     */
    public void resetAlignment() {
        alignController.reset();
    }
    
    // LED control
    public void setLEDOn() {
        limelightTable.getEntry("ledMode").setNumber(3);
    }
    
    public void setLEDOff() {
        limelightTable.getEntry("ledMode").setNumber(1);
    }
}