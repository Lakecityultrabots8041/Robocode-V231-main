package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * RobotState - Centralized state tracking for the robot
 * 
 * This singleton tracks all robot state including:
 * - Pose and velocities (from drivetrain)
 * - Vision targets and alignment status
 * - Mechanism positions (elevator, arms, etc.)
 * 
 * Benefits of this pattern:
 * - No circular dependencies between subsystems
 * - Any subsystem can query state without needing references
 * - Easy to log all state in one place
 * - Clean separation of state vs. control
 */
public class RobotState {
    private static RobotState instance;
    
    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
    
    // ========== DRIVETRAIN STATE ==========
    private final AtomicReference<Pose2d> fieldToRobot = new AtomicReference<>(new Pose2d());
    private final AtomicReference<ChassisSpeeds> measuredSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredSpeeds = new AtomicReference<>(new ChassisSpeeds());
    
    // ========== VISION STATE ==========
    private final AtomicBoolean hasVisionTarget = new AtomicBoolean(false);
    private final AtomicReference<Double> visionTargetX = new AtomicReference<>(0.0);
    private final AtomicReference<Double> visionTargetY = new AtomicReference<>(0.0);
    private final AtomicReference<Integer> visionTargetID = new AtomicReference<>(-1);
    private final AtomicBoolean isAlignedToTarget = new AtomicBoolean(false);
    
    // ========== MECHANISM STATE ==========
    // Elevator
    private final AtomicReference<Double> elevatorHeightRotations = new AtomicReference<>(0.0);
    private final AtomicBoolean elevatorAtTarget = new AtomicBoolean(false);
    
    // Algae Arm
    private final AtomicReference<Double> algaeArmPositionRotations = new AtomicReference<>(0.0);
    private final AtomicBoolean algaeArmAtTarget = new AtomicBoolean(false);
    
    // Coral Arm
    private final AtomicBoolean hasCoralPiece = new AtomicBoolean(false);
    
    private RobotState() {
        // Private constructor for singleton
    }
    
    // ========== DRIVETRAIN UPDATES ==========
    
    public void updatePose(Pose2d pose) {
        fieldToRobot.set(pose);
    }
    
    public void updateMeasuredSpeeds(ChassisSpeeds speeds) {
        measuredSpeeds.set(speeds);
    }
    
    public void updateDesiredSpeeds(ChassisSpeeds speeds) {
        desiredSpeeds.set(speeds);
    }
    
    public Pose2d getFieldToRobot() {
        return fieldToRobot.get();
    }
    
    public ChassisSpeeds getMeasuredSpeeds() {
        return measuredSpeeds.get();
    }
    
    public ChassisSpeeds getDesiredSpeeds() {
        return desiredSpeeds.get();
    }
    
    public Rotation2d getRobotHeading() {
        return fieldToRobot.get().getRotation();
    }
    
    // ========== VISION UPDATES ==========
    
    public void updateVisionTarget(boolean hasTarget, double targetX, double targetY, int targetID) {
        hasVisionTarget.set(hasTarget);
        visionTargetX.set(targetX);
        visionTargetY.set(targetY);
        visionTargetID.set(targetID);
    }
    
    public void setAlignedToTarget(boolean aligned) {
        isAlignedToTarget.set(aligned);
    }
    
    public boolean hasVisionTarget() {
        return hasVisionTarget.get();
    }
    
    public double getVisionTargetX() {
        return visionTargetX.get();
    }
    
    public double getVisionTargetY() {
        return visionTargetY.get();
    }
    
    public int getVisionTargetID() {
        return visionTargetID.get();
    }
    
    public boolean isAlignedToTarget() {
        return isAlignedToTarget.get();
    }
    
    // ========== MECHANISM UPDATES ==========
    
    // Elevator
    public void updateElevatorHeight(double heightRotations) {
        elevatorHeightRotations.set(heightRotations);
    }
    
    public void setElevatorAtTarget(boolean atTarget) {
        elevatorAtTarget.set(atTarget);
    }
    
    public double getElevatorHeight() {
        return elevatorHeightRotations.get();
    }
    
    public boolean isElevatorAtTarget() {
        return elevatorAtTarget.get();
    }
    
    // Algae Arm
    public void updateAlgaeArmPosition(double positionRotations) {
        algaeArmPositionRotations.set(positionRotations);
    }
    
    public void setAlgaeArmAtTarget(boolean atTarget) {
        algaeArmAtTarget.set(atTarget);
    }
    
    public double getAlgaeArmPosition() {
        return algaeArmPositionRotations.get();
    }
    
    public boolean isAlgaeArmAtTarget() {
        return algaeArmAtTarget.get();
    }
    
    // Coral
    public void setHasCoralPiece(boolean hasPiece) {
        hasCoralPiece.set(hasPiece);
    }
    
    public boolean hasCoralPiece() {
        return hasCoralPiece.get();
    }
    
    // ========== UTILITY METHODS ==========
    
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
    
    /**
     * Log all state to SmartDashboard for debugging
     * Call this from Robot.robotPeriodic()
     */
    public void updateDashboard() {
        // Drivetrain
        SmartDashboard.putString("RobotState/Pose", String.format("(%.2f, %.2f) %.1f°",
            fieldToRobot.get().getX(),
            fieldToRobot.get().getY(),
            fieldToRobot.get().getRotation().getDegrees()));
        
        // Vision
        SmartDashboard.putBoolean("RobotState/Has Vision Target", hasVisionTarget.get());
        SmartDashboard.putNumber("RobotState/Vision X", visionTargetX.get());
        SmartDashboard.putNumber("RobotState/Vision Y", visionTargetY.get());
        SmartDashboard.putNumber("RobotState/Vision ID", visionTargetID.get());
        SmartDashboard.putBoolean("RobotState/Is Aligned", isAlignedToTarget.get());
        
        // Mechanisms
        SmartDashboard.putNumber("RobotState/Elevator Height", elevatorHeightRotations.get());
        SmartDashboard.putBoolean("RobotState/Elevator At Target", elevatorAtTarget.get());
        SmartDashboard.putNumber("RobotState/Algae Arm Pos", algaeArmPositionRotations.get());
        SmartDashboard.putBoolean("RobotState/Has Coral", hasCoralPiece.get());
    }
}