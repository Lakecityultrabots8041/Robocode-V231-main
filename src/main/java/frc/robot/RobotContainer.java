// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// ---------- WPILIB IMPORTS ----------
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// ------  SMART DASHBOARD IMPORTS  ------
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// ---------- DRIVE TRAIN IMPORTS ----------
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// ----------- ELEVATOR IMPORTS -----------
import frc.robot.subsystems.elevator.elevator;  //This imports the elevator subsystem and the elevator command
import frc.robot.commands.*; // This Imports all commands from the folder commands
import frc.robot.constants.ElevatorConstants;

// ----------- ARM IMPORTS -----------
import frc.robot.subsystems.arm.AlgaeIntakeSub;
import frc.robot.subsystems.arm.EjectCommandSub; // These import the arm subsystem



public class RobotContainer {
    // Setup for drivetrain subsystem
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Swerve Drivetrain bindings and things */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors **IMPORTANT**
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CommandXboxController controller = new CommandXboxController(0); // Controller setup

     // --------  Setup for elevator subsystem -----------------------------------------------------------------------------------------------------------------------
     private final elevator elevator = new elevator();

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------------

     
  
    // ----- ARM SUBSTEMS ----------------------------------------------------------------------------------------------------------------------------------------------
    private final AlgaeIntakeSub arm = new AlgaeIntakeSub();
    private final EjectCommandSub exitarm = new EjectCommandSub();
    private final AlgaeIntake AlgaeIntake = new AlgaeIntake(arm, 0.5);// Here, the arm will run at 50% speed. Adjust the value as needed.
    private final EjectCommand EjectCommand = new EjectCommand(exitarm, -0.5);  // <----- 50% speed Change here if needed
    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    private final SendableChooser<Command> autoChooser; // *Path Follower*

    public RobotContainer() {
        registerNamedCommands(); //Need to have this to register the named commands for auton to use commands in project

        // ── Autonomous Display For SMARTBOARD ──
        autoChooser = AutoBuilder.buildAutoChooser("Auto Test"); // This is the name of the auto mode that will be displayed on the SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();

    }

    // ── Autonomous Option ──
    private void registerNamedCommands() { //Remember a void is a function that does not return anything, this is a statement to register the named commands 
        NamedCommands.registerCommand("MoveElevatorPosition", new MoveElevator(elevator, 1000.0));

        // Add more commands here when built in path planner just like above, MoveElevatorPosition is what was named in pathPlanner event mode)
    }
    
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // ---- DRIVETRAIN BINDINGS -----------------------------------------------------------------------------------------------------------------------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 2) // Adjust / 2 to change speed
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 2) // Adjust / 2 to change speed
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate / 3) // Drive counterclockwise with negative X (left)
            )
        );

        
        controller.a().whileTrue(drivetrain.applyRequest(() -> brake)); //Emergency Brake
         
        
        // Point mode on right stick button press
        controller.rightStick().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));
        // ------------------------------------------------------------------------------------------------------------------------------------------------------------

        
        // -------- Arm Command Button ASSIGNMENTS ---------------------------------------------------
        controller.x().whileTrue(AlgaeIntake); //<---- Change this to whatever button you wish to use
        controller.b().whileTrue(EjectCommand); //<---- Change this to whatever button you wish to use
        // --------------------------------------------------------------------------------------------


        /* -----------ELEVATOR BINDINGS ----------------------------------------------------------------- */
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
        // -------------------------------------------------------------------------------------------

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
        /* Run the path selected from the auto chooser */
            return autoChooser.getSelected();
        }
    }
