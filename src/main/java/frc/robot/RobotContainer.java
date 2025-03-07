// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// ---------- WPILIB IMPORTS ----------
import static edu.wpi.first.units.Units.*;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// ------  SMART DASHBOARD IMPORTS  ------
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

// ---------- DRIVE TRAIN IMPORTS ----------
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


import frc.robot.Constants.*;

// ---------- SUBSYSTEM IMPORTS ----------
import frc.robot.subsystems.algae_arm.Algae_Subsystem;
import frc.robot.subsystems.coral.Coral_Subsystem;
//import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
//import frc.robot.subsystems.algae_arm.Algae_EjectCommand_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;

//import frc.robot.commands.ELEVATOR.MoveElevatorDown_cmd;
//import frc.robot.commands.ELEVATOR.MoveElevatorUp_cmd;
//import frc.robot.commands.ELEVATOR.MoveElevatorDownFast_cmd;
//import frc.robot.commands.ELEVATOR.MoveElevatorUpFast_cmd;

//--------------------COMMANDS-----------------------------
import frc.robot.commands.UltrabotsCommand;
//import frc.robot.commands.ALGAE_ARM.AA_ArmLift_cmd;
//import frc.robot.commands.Unused_Commands.*;
//import frc.robot.commands.ALGAE_ARM.AA_Eject_cmd;
//import frc.robot.commands.ALGAE_ARM.AA_Intake_cmd;



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


    //===========================SUBSYSTEMS=======================================================================================================================
     // Initialize subsystems using the new refactored approach
    private final Algae_Subsystem algaeSubsystem = new Algae_Subsystem();
    private final Coral_Subsystem coralSubsystem = new Coral_Subsystem();
    private final Elevator_Subsystem elevatorSubsystem = new Elevator_Subsystem();

    //===========================AUTON PATH CALLER=========================================================================================================================/
    private final SendableChooser<Command> autoChooser; // SmartDashboard chooser for autonomous mode

    public RobotContainer() {

        //Register commands for autonomous
        registerAutonomousCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Robot Mid"); // This is where you change our autonomous mode durning competition, techincain will change this depending on strategy
        SmartDashboard.putData("Auton Mode", autoChooser);
        
        configureBindings();
    }


    private void registerAutonomousCommands() {

        NamedCommands.registerCommand("Smelevator", // Recommened to change this to call elevator or something, score l1 and launch coral is what should happen. Drives straight at reef. 
        UltrabotsCommand.autonomousL1Score(elevatorSubsystem, coralSubsystem));

        // Register L1/L2/L3 scoring commands
        NamedCommands.registerCommand("ScoreL1", //Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousL1Score(elevatorSubsystem, coralSubsystem));
        NamedCommands.registerCommand("ScoreL2", // Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousL2Score(elevatorSubsystem, coralSubsystem));
        NamedCommands.registerCommand("ScoreL3", // Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousL3Score(elevatorSubsystem, coralSubsystem));
            
        // Register setup positions
        NamedCommands.registerCommand("StationSetup", // Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousStationSetup(elevatorSubsystem));
        NamedCommands.registerCommand("ProcessorHeight", // Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousProcessorHeight(elevatorSubsystem));
            
        // Register algae collection
        NamedCommands.registerCommand("CollectAlgae", // Name you use in pathplanner to call this command
            UltrabotsCommand.autonomousCollectAlgae(elevatorSubsystem, algaeSubsystem));
    }

    
    public Elevator_Subsystem getElevator() {
        return elevatorSubsystem;
    }


    private void configureBindings() {
        // ---- DRIVETRAIN BINDINGS -----------------------------------------------------------------------------------------------------------------------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 3.5) // Adjust / 3 to change speed
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 3) // Adjust / 2 to change speed
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    //------------ALAGE Configurations------------------------------------------------------------------------------------------------------------------------


         // ALGAE Intake/Outake
        controller.leftTrigger().whileTrue(algaeSubsystem.intakeCommand(0.5));
                
        controller.rightTrigger().whileTrue(algaeSubsystem.ejectCommand(0.5));


        // ALGAE ARM Controls
        controller.povLeft().onTrue(algaeSubsystem.setArmHeightCommand(AlgaeArmConstants.ARM_STOWED_POSITION));
        controller.povRight().onTrue(algaeSubsystem.setArmHeightCommand(AlgaeArmConstants.ARM_LOWER_POSITION));

    
    //------------CORAL Configurations------------------------------------------------------------------------------------------------------------------------

        controller.leftBumper().and(controller.x()).whileTrue(coralSubsystem.getIntakeCommand(0.2)); // set speed either here or in Constants
                
        controller.x().whileTrue(coralSubsystem.getOutakeCommand(0.2)); // Set speed either here or in Constants


    //------------ELEVATOR Button Assignments------------------------------------------------------------------------------------------------------------------------
    
        controller.povUp().whileTrue(elevatorSubsystem.getManualUpCommand());
        controller.povDown().whileTrue(elevatorSubsystem.getManualDownCommand());

        // Faster Manual Elevator Control      
        controller.leftBumper().and(controller.povUp()).whileTrue(elevatorSubsystem.getManualUpFastCommand());        
        controller.leftBumper().and(controller.povDown()).whileTrue(elevatorSubsystem.getManualDownFastCommand());


         // ---- SCORING PRESET BUTTONS ----
        controller.a().onTrue(UltrabotsCommand.prepareToScoreL1(elevatorSubsystem));
        controller.b().onTrue(UltrabotsCommand.prepareToScoreL2(elevatorSubsystem));
        controller.y().onTrue(UltrabotsCommand.prepareToScoreL3(elevatorSubsystem));



       /*  // ---- SYSID / FIELD-CENTRIC BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */ 
 

        // reset the field-centric heading on right bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }


       public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

}

       