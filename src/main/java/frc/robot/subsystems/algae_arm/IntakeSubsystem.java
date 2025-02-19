package frc.robot.subsystems.algae_arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_VictorSPX leftIntakeMotor; //updated to WPI_VictorSPX
    private final WPI_VictorSPX rightIntakeMotor; //updated to WPI_VictorSPX


    public IntakeSubsystem() {
        // Initialize motors with correct CAN IDs
        leftIntakeMotor = new WPI_VictorSPX(4); //verify CAN ID
        rightIntakeMotor = new WPI_VictorSPX(2); //verify CAN ID
        
        configureMotors();
    }

    private void configureMotors() {
        // Set motors to Brake mode for precise control
        leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
        rightIntakeMotor.setNeutralMode(NeutralMode.Brake);

        // Configure right motor to follow the left motor
        rightIntakeMotor.follow(leftIntakeMotor);
        rightIntakeMotor.setInverted(true);  // Change if set incorrectly

    }

    public void runIntake(double speed) {
        // Control left motor, and right motor follows
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
        
    }
    
    public void stop() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);  // Stop both motors
    }

    @Override
    public void periodic() {
        // Log motor outputs instead of positions as you can't with this style of motor:(
        Logger.recordOutput("arm/LeftMotorOutput", leftIntakeMotor.getMotorOutputPercent());
        Logger.recordOutput("arm/RightMotorOutput", rightIntakeMotor.getMotorOutputPercent());
    }
}
