// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;

public class FalconStop extends CommandBase {
  
  private FalconController motor;
  private final FileLog log;

  /**
   * Stops the motor
   * @param motor motor subsystem
   * @param log logfile
   */
  public FalconStop(FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, motor.getName(), "Stop");
    motor.stopMotor();
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
