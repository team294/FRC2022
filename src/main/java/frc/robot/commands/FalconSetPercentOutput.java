// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class FalconSetPercentOutput extends CommandBase {

  private final FalconController motor;
  private final FileLog log;
  private double percentOut = 0.0;
  private boolean fromShuffleboard = false;

  /**
   * Sets the motor speed from Shuffboard, -1.0 to +1.0
   * @param motor motor subsystem
   * @param log logfile
   */
  public FalconSetPercentOutput(FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    fromShuffleboard = true;

    SmartDashboard.putNumber(buildString(motor.getName(), " Percent"), 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  /**
   * Sets the motor speed
   * @param percentOut -1.0 to +1.0
   * @param motor motor subsystem
   * @param log logfile
   */
  public FalconSetPercentOutput(double percentOut, FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    this.percentOut = percentOut;
    fromShuffleboard = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      percentOut = SmartDashboard.getNumber(buildString(motor.getName(), " Percent"), 0);
    } else {
      SmartDashboard.putNumber(buildString(motor.getName(), " Percent"), percentOut);
    }

    log.writeLog(false, motor.getName(), "SetPercentOutput", "Percent", percentOut);
    motor.setMotorPercentOutput(percentOut);
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
