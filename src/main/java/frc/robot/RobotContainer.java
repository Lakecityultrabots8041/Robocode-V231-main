// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// ---------- WPILIB IMPORTS ----------
import static edu.wpi.first.units.Units.*;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.coral_arm.Coral_ArmLift_Sub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Ratchet_Climber.Ratchet_Subsystem;
import frc.robot.subsystems.algae_arm.Algae_Intake_Sub;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.algae_arm.Algae_EjectCommand_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
import frc.robot.commands.*; // This Imports all commands from the folder commands
//import frc.robot.constants.AlgaeArm_Constants;
import frc.robot.constants.Elevator_Constants;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;



public class RobotContainer {
    
    //Setup Command Xbox Controller
    private final CommandXboxController controller = new CommandXboxController(0); 

    //--------------------DRIVETRAIN SETUP------------------------------------------------------------------------------------------------------------------------
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open loop voltage control over closed loop control
    
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    ////--------------------ARM SETUP---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
   
    // Setup for arm subsystem
    private final Algae_ArmLift_Sub algae_arm = new Algae_ArmLift_Sub();
    private final Algae_Intake_Sub algae_intake = new Algae_Intake_Sub();
    private final Algae_EjectCommand_Sub algae_deploy = new Algae_EjectCommand_Sub();
    
    // Setup for arm intake and eject commands
    private final AA_Intake_cmd AlgaeIntake = new AA_Intake_cmd(algae_intake, 0.5);
    private final AA_Eject_cmd EjectCommand = new AA_Eject_cmd(algae_deploy, -0.5); 
     
    ////--------------------------------------ELEVATOR SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    
    // Setup for elevator subsystem    
    private final Elevator_Subsystem elevator = new Elevator_Subsystem();

    // Assign Home position to elevator with home constant
    private final SetElevatorLevel setElevator_Home = new SetElevatorLevel(elevator, Elevator_Constants.Home_Position);
    
    
    
    ////--------------------------------------CORAL ARM SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    //Declare the Coral Arm subsystem
    private final Coral_ArmLift_Sub coral_arm = new Coral_ArmLift_Sub();
    private final Coral_Arm_Intake_Sub intakeMotor = new Coral_Arm_Intake_Sub();

    // Setup intake and eject commands for Coral
    private final CA_WheelIn_cmd ca_wheelin_cmd = new CA_WheelIn_cmd(intakeMotor, 0.05);
    private final CA_WheelOut_cmd ca_wheelout_cmd = new CA_WheelOut_cmd(intakeMotor, 0.05);
    private final CA_Intake_cmd ca_intake_command = new CA_Intake_cmd(elevator, coral_arm, algae_arm);


    
    // Setup for Scoring Commands for Coral and Algae using elevator
    private final Move_L3_Score move_L3_score = new Move_L3_Score(elevator, coral_arm, algae_arm);
    private final Move_L2_Score move_L2_score = new Move_L2_Score(elevator, coral_arm, algae_arm);
   // private final Move_L1_Score move_L1_score = new Move_L1_Score(elevator, coral_arm, algae_arm);

    
    ////--------------------------------------Ratchet SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    //private final Ratchet_Subsystem ratchet = new Ratchet_Subsystem();
    // private final Ratchet_set_cmd toggleRatchet =
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
    private final SendableChooser<Command> autoChooser; // *Path Follower*

    public RobotContainer() {
        registerNamedCommands(); //Need to have this to register the named commands for auton to use commands in project

        // ── Autonomous Display For SMARTBOARD ──
        autoChooser = AutoBuilder.buildAutoChooser("Auto Test_Center Blue"); // This is the name of the auto mode that will be displayed on the SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    // ── Autonomous Option ──
    private void registerNamedCommands() { //Remember a void is a function that does not return anything, this is a statement to register the named commands 
        NamedCommands.registerCommand("MoveElevatorPosition", new SetElevatorLevel(elevator, 1.0));

    
    }
    
    public Elevator_Subsystem getElevator() {
        return elevator;
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // ---- DRIVETRAIN BINDINGS -----------------------------------------------------------------------------------------------------------------------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 2) // Adjust / 2 to change speed
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 2) // Adjust / 2 to change speed
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        // Choose a button below to handle emergency brake
        //controller.a().whileTrue(drivetrain.applyRequest(() -> brake)); //Emergency Brake
        // Point mode on right stick button press
        //controller.b().whileTrue(drivetrain.applyRequest(() ->
            //point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        
        // -------- ALGAE Intake/Outake Commands---------------------------------------------------
        controller.leftTrigger().toggleOnTrue(AlgaeIntake); 
        controller.rightTrigger().toggleOnFalse(EjectCommand);

        // --------------------------------------------------------------------------------------------
        //Coral Intake/Outake Commands
        controller.povRight().whileTrue(ca_wheelin_cmd);
        controller.povLeft().whileTrue(ca_wheelout_cmd);

        // ---- Scoring Button Controls---------------------------------------------------------------------
        //controller.a().onTrue(move_L1_score); // sets elevator to L1 score position
        controller.b().onTrue(move_L2_score);   // Set off chain to score L2
        controller.y().onTrue(move_L3_score);  // Set off chain to score coral and take algae
        controller.povDown().onTrue(setElevator_Home); // Set elevator to home position
        controller.leftBumper().onTrue(ca_intake_command);  // Set off chain to get coral from station

        // -------------------------------------------------------------------------------------------
         // ---- RATCHET BINDINGS ---------------------------------------------------------------------
        
        // controller.x().onTrue(toggleRatchet);
        

         // ---- SYSID / FIELD-CENTRIC BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
 

        // reset the field-centric heading on left bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
        public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
            return autoChooser.getSelected();
        }
    }
