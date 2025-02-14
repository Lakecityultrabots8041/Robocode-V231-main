// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MoveElevator;



public class Robot extends LoggedRobot {

  
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private final Map<String, Command> autoEventMap = new HashMap<>();

  public Robot() {     
    m_robotContainer = new RobotContainer();
    configureAutoEvents();
  }

    private void configureAutoEvents(){
      autoEventMap.put(
        "MoveElevatorToPosition",
        new MoveElevator(m_robotContainer.getElevator(), 1000.0)  // Example target position
    );
    
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

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

    // Get trigger inputs for elevator control
    double upTrigger = m_robotContainer.getController().getRightTriggerAxis();
    double downTrigger = m_robotContainer.getController().getLeftTriggerAxis();

    // Calculate elevator voltage (-12V to 12V range)
    double elevatorVoltage = (upTrigger - downTrigger) * 12.0;

    // Apply voltage to the elevator
    m_robotContainer.getElevator().runVolts(elevatorVoltage);

    // Log applied voltage for debugging
    Logger.recordOutput("Elevator/AppliedVoltage", elevatorVoltage);



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
