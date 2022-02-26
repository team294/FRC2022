// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallLocation;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class BallCountAddBall extends CommandBase {
  /** Creates a new AddBall. */
  private BallLocation location;
  private FileLog log;

  public BallCountAddBall(BallLocation location, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.location = location;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BallCount.addBall(location, log);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
