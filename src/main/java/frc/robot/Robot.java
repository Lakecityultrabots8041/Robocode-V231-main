// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;



import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends LoggedRobot {

  
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  
  // Drive Motor Assignments for bad auton
  //private final TalonFX flDrive = new TalonFX(10);
  //private final TalonFX frDrive = new TalonFX(20);
  //private final TalonFX blDrive = new TalonFX(30);
  //private final TalonFX brDrive = new TalonFX(40);


  //private final SendableChooser<Command> autoChooser; // *Path Follower*

  public Robot() {     
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    configureAutoEvents();
  }

  private void configureAutoEvents() {
     // autoEventMap.put("DriveForward", new DriveForward(m_robotContainer.getDriveTrain()));
    // autoEventMap.put("DriveBackward", new DriveBackward(m_robotContainer.getDriveTrain()));
    // autoEventMap.put("TurnLeft", new TurnLeft(m_robotContainer.getDriveTrain()));
    // autoEventMap.put("TurnRight", new TurnRight(m_robotContainer.getDriveTrain()));
  }
  @Override
  public void robotInit() {
      // Set up the logger for data recording
      super.robotInit();
      Logger.recordMetadata("ProjectName", "Elevator Simulation");
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));  // Save logs to USB on the RoboRIO
      Logger.addDataReceiver(new NT4Publisher());  // Stream data live to NetworkTables
      Logger.start();

      
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    Logger.recordOutput("Robot/Status", "Running");
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}
  
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    //bad auton
    /*
    flDrive.set(0.2);
    frDrive.set(0.2);
    blDrive.set(0.2);
    brDrive.set(0.2);
    */
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
     // Log elevator height for debugging
     Logger.recordOutput("Elevator/CurrentHeight", m_robotContainer.getElevator().getCurrentHeight());
 }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
