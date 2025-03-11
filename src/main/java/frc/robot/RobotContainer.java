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
import frc.robot.subsystems.algae_arm.Algae_Intake_Sub;
import frc.robot.subsystems.algae_arm.Algae_ArmLift_Sub;
import frc.robot.subsystems.algae_arm.Algae_EjectCommand_Sub;
import frc.robot.subsystems.elevator.Elevator_Subsystem;
import frc.robot.constants.AlgaeArm_Constants;
import frc.robot.constants.Elevator_Constants;
import frc.robot.subsystems.coral_arm.Coral_Arm_Intake_Sub;
import frc.robot.subsystems.coral_arm.Coral_Arm_Out_Sub;
import frc.robot.commands.ELEVATOR.MoveElevatorDown_cmd;
import frc.robot.commands.ELEVATOR.MoveElevatorUp_cmd;
import frc.robot.commands.ELEVATOR.MoveElevatorDownFast_cmd;
import frc.robot.commands.ELEVATOR.MoveElevatorUpFast_cmd;
import frc.robot.commands.ALGAE_ARM.AA_ArmLift_cmd;
import frc.robot.commands.Unused_Commands.*;
import frc.robot.commands.ALGAE_ARM.AA_Eject_cmd;
import frc.robot.commands.ALGAE_ARM.AA_Intake_cmd;
import frc.robot.commands.Auton_Commands.AUTON_CMD;
import frc.robot.commands.ELEVATOR.SetElevatorLevel;



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
      
       //----------Algae Arm Commands---------------------------------------------------------------------------------------------------------------------------
    private final AA_ArmLift_cmd algaeArmUp = new AA_ArmLift_cmd(algae_arm, AlgaeArm_Constants.ARM_UPPER_POSITION);
    private final AA_ArmLift_cmd algaeArmDown = new AA_ArmLift_cmd(algae_arm, AlgaeArm_Constants.ARM_LOWER_POSITION);
    
    // Setup for arm intake and eject commands
    private final AA_Intake_cmd AlgaeIntake = new AA_Intake_cmd(algae_intake, 0.5);
    private final AA_Eject_cmd EjectCommand = new AA_Eject_cmd(algae_deploy, -0.5); 
     
    ////--------------------------------------ELEVATOR SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    
    // Setup for elevator subsystem    
    private final Elevator_Subsystem elevator = new Elevator_Subsystem();

    private final SetElevatorLevel L2 = new SetElevatorLevel(elevator, Elevator_Constants.L2_Middle_Score);
    private final SetElevatorLevel L3 = new SetElevatorLevel(elevator, Elevator_Constants.L3_TOP_Score);

    private final SetElevatorLevel ALG_L2 = new SetElevatorLevel(elevator, Elevator_Constants.AlGAE_L2_SCORE);
    private final SetElevatorLevel ALG_L3 = new SetElevatorLevel(elevator, Elevator_Constants.ALGAE_L3_SCORE);

    private final SetElevatorLevel Proc_LVL = new SetElevatorLevel(elevator, Elevator_Constants.Processor_Position);
    
    
    // Assign Home position to elevator with home constant
    //private final SetElevatorLevel setElevator_Home = new SetElevatorLevel(elevator, Elevator_Constants.Home_Position);
    //The above is used for the old elevator code, ignore unless we revert


    
    
    ////--------------------------------------CORAL ARM SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    //Declare the Coral Arm subsystem
    //private final Coral_ArmLift_Sub coral_arm = new Coral_ArmLift_Sub();

    //Setup for Coral Arm Commands
    private final Coral_Arm_Intake_Sub intakeMotor = new Coral_Arm_Intake_Sub();
    private final Coral_Arm_Out_Sub OutMotor = new Coral_Arm_Out_Sub();

    // Setup intake and eject commands for Coral
    private final CA_WheelIn_cmd ca_wheelin_cmd = new CA_WheelIn_cmd(OutMotor, 0.10);
    private final CA_WheelOut_cmd ca_wheelout_cmd = new CA_WheelOut_cmd(intakeMotor, -0.20);
    //private final CA_Intake_cmd ca_intake_command = new CA_Intake_cmd(elevator, coral_arm, algae_arm);





    //Don't like having to set speed here, but it's the only way to get the command to work
    // Setup for Scoring Commands for Coral and Algae using elevator

    private final MoveElevatorUp_cmd moveElevatorup = new MoveElevatorUp_cmd(elevator, 0.3);
    private final MoveElevatorDown_cmd moveElevatordown = new MoveElevatorDown_cmd(elevator, 0.3);
    private final MoveElevatorUpFast_cmd moveElevatorupF = new MoveElevatorUpFast_cmd(elevator, 0.3);
    private final MoveElevatorDownFast_cmd moveElevatordownF = new MoveElevatorDownFast_cmd(elevator, 0.3);
 
    private final Move_L3_Score move_L3_score = new Move_L3_Score(elevator, intakeMotor);
    private final Move_L2_Score move_L2_score = new Move_L2_Score(elevator, intakeMotor);
    private final Move_L1_Score move_L1_Score  = new Move_L1_Score(elevator, intakeMotor);

    //private final Command m_complexAuto = newComplexAuto(m_robotDrive, m_hatchSubSystem);

   
    private final AUTON_CMD testauto; // change this to reflect main AUTON
    private final AUTON_CMD goleftauto; // use this as a template to define what we want
    private final AUTON_CMD gorightauto; // use this as a template to define what we want
    //private final SendableChooser<Command> autoChooser; // *Path Follower*
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {

        testauto = new AUTON_CMD(elevator, intakeMotor);
        goleftauto = new AUTON_CMD(elevator, intakeMotor);
        gorightauto = new AUTON_CMD(elevator, intakeMotor);


        NamedCommands.registerCommand("Smelevator", testauto);

        m_chooser.setDefaultOption("Robot Mid", testauto);
        m_chooser.addOption("Robot MidLeft", goleftauto);
        m_chooser.addOption("Robot MidRight", gorightauto);
        //m_chooser.addOption("Robot Left", testauto);
       // m_chooser.addOption("Robot Right", testauto);
       // m_chooser.addOption("Big Red", testauto);
        SmartDashboard.putData("Auton Mode", m_chooser);
       
        configureBindings();
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
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed / 3) // Adjust / 3 to change speed
                    .withVelocityY(-controller.getLeftX() * MaxSpeed / 3) // Adjust / 2 to change speed
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    //------------ALAGE Configurations------------------------------------------------------------------------------------------------------------------------
      
        // --------ALGAE Intake/Outake Commands-\\
        controller.leftTrigger().toggleOnTrue(AlgaeIntake); 
        //controller.rightTrigger().whileTrue(EjectCommand);




        // ---- ALGAE ARM BUTTONS--------\\
        controller.povLeft().onTrue(algaeArmUp);  //--dpad left button to raise Algae Arm
        //controller.leftBumper().and(controller.b()).whileTrue(ALG_L2);
        controller.povRight().onTrue(algaeArmDown);  //--dpad right button to lower Algae Arm
    
    //------------CORAL Configurations------------------------------------------------------------------------------------------------------------------------
        
        //Coral Intake/Outake Buttons\\


        controller.leftBumper().and(controller.x()).whileTrue(ca_wheelin_cmd);

        controller.x().whileTrue(ca_wheelout_cmd);

    //------------ELEVATOR Button Assignments------------------------------------------------------------------------------------------------------------------------
    
        controller.povUp().whileTrue(moveElevatorup);
        controller.leftBumper().and(controller.povUp()).whileTrue(moveElevatorupF);
        controller.povDown().whileTrue(moveElevatordown);
        controller.leftBumper().and(controller.povDown()).whileTrue(moveElevatordownF); 

        // ---- Scoring Button Controls---------------------------------------------------------------------
        //controller.a().onTrue(move_L1_score); // sets elevator to L1 score position
        controller.b().onTrue(L2);   // Set off chain to score L2
        controller.y().onTrue(L3);  // Set off chain to score coral and take algae
        controller.leftBumper().and(controller.b()).whileTrue(ALG_L2);
        controller.leftBumper().and(controller.y()).whileTrue(ALG_L3);
        controller.leftBumper().and(controller.rightTrigger()).whileTrue(Proc_LVL);

        

         // ---- SYSID / FIELD-CENTRIC BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
 

        // reset the field-centric heading on right bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }


       public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        
       }

     
       }

