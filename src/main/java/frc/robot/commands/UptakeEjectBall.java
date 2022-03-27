package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeEjectBall extends CommandBase {
  Uptake uptake;
  FileLog log;
  boolean hasPassed;

  /**
   * Ejects a ball from the uptake
   * 
   * @param uptake subsystem
   * @param log file logger
   */
  public UptakeEjectBall(Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.log = log;
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "UptakeEjectBall", "Init");
    hasPassed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    uptake.setEjectPercentOutput(0.25);
    if(uptake.isBallGoingToFeeder()){
      hasPassed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uptake.setEjectPercentOutput(0);
    log.writeLog(false, "UptakeEjectBall", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hasPassed && !uptake.isBallGoingToFeeder()){
      return true;
    }
    else return false;
  }
}
