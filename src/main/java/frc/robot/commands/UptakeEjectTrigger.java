package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class UptakeEjectTrigger extends CommandBase {
  Uptake uptake;
  FileLog log;

  /**
   * Called when eject sensor is triggered 
   * 
   * @param uptake subsystem
   * @param log file logger
   */
  public UptakeEjectTrigger(Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.log = log;
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "UptakeEjectTrigger", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    uptake.setEjectPercentOutput(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check to see if ball is still in ejector
    if (!uptake.isBallInEjector()){
      uptake.setEjectPercentOutput(0);
      BallCount.setBallCount(0, BallLocation.kEject, log);
      log.writeLog(false, "UptakeEjectTrigger", "End");
      return true;
    }
    else return false;
  }
}
