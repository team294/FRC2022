package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeToggleAlliance extends CommandBase {
  Uptake uptake;
  FileLog log;
  boolean hasPassed;

  /**
   * Toggles the alliance used by the uptake
   * 
   * @param uptake subsystem
   * @param log file logger
   */
  public UptakeToggleAlliance(Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.log = log;
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "UptakeToggleAlliance", "Init");
    uptake.toggleAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return true;
  }
}
