// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class EjectBall extends CommandBase {
  Uptake uptake;
  FileLog log;
  boolean hasPassed;
  /** Creates a new EjectBall. */
  public EjectBall(Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.log = log;
    addRequirements(uptake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasPassed = false;
    BallCount.setBallCount(0, BallLocation.kUptake, log);
    BallCount.setBallCount(1, BallLocation.kEject, log);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    uptake.setEjectPercentOutput(-0.25);
    if(uptake.isBallInEjector()){
      hasPassed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hasPassed && !uptake.isBallInEjector()){
      uptake.setEjectPercentOutput(0);
      BallCount.setBallCount(0, BallLocation.kEject, log);
      return true;
    }
    else return false;
  }
}
