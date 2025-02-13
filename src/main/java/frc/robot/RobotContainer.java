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


/* SMART DASHBOARD IMPORTS */
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* PATH PLANNER IMPORTS*/ 
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.elevator.elevator;  //This imports the elevator subsystem and the elevator command
import frc.robot.commands.*; // This Imports all commands from the folder commands
import frc.robot.constants.ElevatorConstants;

import frc.robot.subsystems.arm.AlgaeIntakeSub;
import frc.robot.subsystems.arm.EjectCommandSub; // These import the arm subsystem


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    // Setup for elevator subsystem
     private final elevator elevator = new elevator();

    // Setup for controller for all systems(arm, elevator, drivetrain, ect.)
     private final CommandXboxController controller = new CommandXboxController(0);

     /* SET UP FOR PATH PLANNER */
     private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Setup for arm subsystem
    private final AlgaeIntakeSub arm = new AlgaeIntakeSub();
    private final EjectCommandSub exitarm = new EjectCommandSub();
    // Here, the arm will run at 50% speed. Adjust the value as needed.
    private final AlgaeIntake AlgaeIntake = new AlgaeIntake(arm, 0.5);
    private final EjectCommand EjectCommand = new EjectCommand(exitarm, -0.5);  // <----- 50% speed Change here if needed


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

        //????
        controller.y().whileTrue(AlgaeIntake);
        
         //<---- Change this to whatever button you wish to use
        controller.x().whileTrue(EjectCommand); //<---- Change this to whatever button you wish to use
        
        // Alternatively, if you prefer the arm command to run continuously (default command),
        // you can set it as the default command for the Arm subsystem:
        // arm.setDefaultCommand(armCommand);

        configureAuto();
    }
    private void configureAuto() {
        // Load Paths from PathPlanner
        //Command centerBlueAuto = AutoBuilder.buildAuto("Center Blue Part 1"); // Name matches JSON file
        //Command part2auto = AutoBuilder.buildAuto("Part 2"); // Rename in pathplanner and here to matach above
        //Command part3auto = AutoBuilder.buildAuto("Part 3"); // Rename in pathplanner and here to matach above
        //Command part4auto = AutoBuilder.buildAuto("Part 4"); // Rename in pathplanner and here to matach above
        //Command part5auto = AutoBuilder.buildAuto("Part 5"); // Rename in pathplanner and here to matach above
        //Command part6auto = AutoBuilder.buildAuto("Part 6"); // Rename in pathplanner and here to matach above
    // Add Autonomous Commands to the SendableChooser
        //autonChooser.setDefaultOption("Center Blue Auto", centerBlueAuto);
        //autonChooser.addOption("Another Auto", part2auto);
        //autonChooser.addOption("Another Auto", part3auto);
        //autonChooser.addOption("Another Auto", part4auto);
        //autonChooser.addOption("Another Auto", part5auto);
        //autonChooser.addOption("Another Auto", part6auto);

        // Display the auton chooser on SmartDashboard
        SmartDashboard.putData("Autonomous Mode", autonChooser);
        
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


        // ---- DRIVETRAIN BINDINGS ----
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 2) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 2) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate / 3) // Drive counterclockwise with negative X (left)
            )
        );

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        /* -----------ELEVATOR BINDINGS -------------- */

        controller.getLeftTriggerAxis();
        new Trigger(() -> controller.getLeftTriggerAxis() > 0.1)
            .onTrue(new AdjustElevator(elevator, -0.05)); 

        new Trigger(() -> controller.getRightTriggerAxis() > 0.1)
            .onTrue(new AdjustElevator(elevator, 0.05));   // Raise elevator by 0.05 m


         /* new Trigger(() -> controller.getLeftTriggerAxis() > 0.1)
         .whileTrue(new MoveElevator(elevator, 0));  // Lower elevator with Left Trigger

         new Trigger(() -> controller.getRightTriggerAxis() > 0.1)
         .whileTrue(new MoveElevator(elevator, 20000));  // Raise elevator with Right Trigger */
        
         // Bind D-pad directions to preset elevator positions:
        controller.povUp().onTrue(new SetElevatorLevel(elevator, ElevatorConstants.LEVEL_4));
        controller.povRight().onTrue(new SetElevatorLevel(elevator, ElevatorConstants.LEVEL_3));
        controller.povLeft().onTrue(new SetElevatorLevel(elevator, ElevatorConstants.LEVEL_2));
        controller.povDown().onTrue(new SetElevatorLevel(elevator, ElevatorConstants.LEVEL_1));
    
         
        // ---- ARM BINDINGS ----
        // Bind the ArmCommand to the X button
        controller.x().whileTrue(AlgaeIntake);

         // ---- SYSID / FIELD-CENTRIC BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
 

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AlgaeIntake.withTimeout(3) //Change this when we get pathplanner setup
        .andThen(Commands.print("Autonomous complete"));
    }
}
