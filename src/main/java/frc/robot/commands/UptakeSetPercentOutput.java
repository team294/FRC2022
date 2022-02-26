// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeSetPercentOutput extends CommandBase {
  /** Creates a new testUptake. */
  private final Uptake uptake;
  private double percent;
  private FileLog log;
  private boolean ejectBall;

  /**
   * Runs the uptake motors
   * @param percent -1.0 to 1.0, + = up, - = down
   * @param ejectBall true = ball path to eject, false = ball path to feeder
   * @param uptake uptake subsystem
   * @param log logfile
   */
  public UptakeSetPercentOutput(double percent, boolean ejectBall, Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.percent = percent;
    this.log = log;
    this.ejectBall = ejectBall;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    uptake.setMotorPercentOutput(percent ,ejectBall);
    log.writeLog(false, "Uptake Set Percent", "initialize", "percent", percent, "Eject Ball", ejectBall);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
