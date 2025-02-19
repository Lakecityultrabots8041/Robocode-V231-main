package frc.robot.subsystems.algae_arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class EjectCommandSub implements Subsystem {

    private final WPI_VictorSPX leftArmMotor; //updated to WPI_VictorSPX
    private final WPI_VictorSPX rightArmMotor; //updated to WPI_VictorSPX

    public EjectCommandSub() {
        // Initialize motors with correct CAN IDs
        leftArmMotor = new WPI_VictorSPX(4); //verify CAN ID
        rightArmMotor = new WPI_VictorSPX(2); //verify CAN ID
        configureMotors();
    }

    private void configureMotors() {
        // Set motors to Brake mode for precise control
        leftArmMotor.setNeutralMode(NeutralMode.Brake);
        rightArmMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void runArms(double speed) {
        // Control left motor, and right motor follows
        leftArmMotor.set(-speed);
        rightArmMotor.set(-speed);
    }

    public void stop() {
        leftArmMotor.set(0);
        rightArmMotor.set(0);  // Stop both motors
    }

    @Override
    public void periodic() {
        // Log motor outputs instead of positions as you can't with this style of motor:(
        Logger.recordOutput("arm/LeftMotorOutput", leftArmMotor.getMotorOutputPercent());
        Logger.recordOutput("arm/RightMotorOutput", rightArmMotor.getMotorOutputPercent());
    }
}
