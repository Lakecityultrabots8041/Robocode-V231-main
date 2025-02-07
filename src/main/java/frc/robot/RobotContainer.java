// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


//This imports the elevator subsystem and the elevator command
import frc.robot.subsystems.elevator.elevator;
// This imports the arm command
import frc.robot.commands.*; // This Imports all commands from the folder commands
/* Probably Smart to only set above to the exact command wanting to import as this isn't python and java can be dumb */

import frc.robot.subsystems.arm.Arm; // This imports the arm subsystem

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    // Setup for elevator subsystem
     private final elevator elevator = new elevator();

    // Setup for controller for all systems(arm, elevator, drivetrain, ect.)
     private final CommandXboxController controller = new CommandXboxController(0);

    // Setup for arm subsystem
    private final Arm arm = new Arm();
    // Here, the arm will run at 50% speed. Adjust the value as needed.
    private final ArmCommand ArmCommand = new ArmCommand(arm, 0.5); // <----- 50% speed Change here if needed


    /* Swerve Drivetrain bindings and things */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors **IMPORTANT**
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public RobotContainer() {
        configureBindings();
         // ── Teleop Option ──
        // Bind the ArmCommand to the controller's X button so that the command runs while the button is held.

        controller.x().whileTrue(ArmCommand); //<---- Change this to whatever button you wish to use

        // Alternatively, if you prefer the arm command to run continuously (default command),
        // you can set it as the default command for the Arm subsystem:
        // arm.setDefaultCommand(armCommand);
    }


    public elevator getElevator() {
        return elevator;
    }

    public CommandXboxController getController() {
        return controller;
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        //drive train stuff
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 2) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 2) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate / 3) // Drive counterclockwise with negative X (left)
            )
        );
        
        // changed the value rate above
        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));


         // Elevator control with triggers
         new Trigger(() -> controller.getLeftTriggerAxis() > 0.1)
         .whileTrue(new MoveElevator(elevator, 0));  // Lower elevator

         new Trigger(() -> controller.getRightTriggerAxis() > 0.1)
         .whileTrue(new MoveElevator(elevator, 20000));  // Raise elevator (example target)


        // Bind the ArmCommand to a button press
        controller.x().whileTrue(ArmCommand);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return ArmCommand.withTimeout(3) //Change this when pathplanner is created, as this will run for only 3 seconds at the start of switching on Auton
        .andThen(Commands.print("Autonomous complete"));
    }
}
