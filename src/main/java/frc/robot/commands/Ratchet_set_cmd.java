package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ratchet_Climber.Ratchet_Subsystem;

/**
 * A command to set the ratchet servo to a specific position.
 */
public class Ratchet_set_cmd extends Command {
  private final Ratchet_Subsystem m_ratchetSubsystem;
  private final double m_position;

  /**
   * Creates a new command to set the ratchet servo to a specific position.
   *
   *  ratchetSubsystem The ratchet subsystem
   *  position The position to set (0.0 to 1.0)
   */
  public Ratchet_set_cmd(Ratchet_Subsystem ratchetSubsystem, double position) {
    m_ratchetSubsystem = ratchetSubsystem;
    m_position = position;
    addRequirements(ratchetSubsystem);
  }

  
  @Override
  public void initialize() {
    
    double safePosition = m_position;
    if (safePosition < 0.0) safePosition = 0.0;
    if (safePosition > 1.0) safePosition = 1.0;
    
    m_ratchetSubsystem.setPosition(safePosition);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}