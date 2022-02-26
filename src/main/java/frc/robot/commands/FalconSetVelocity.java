
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.buildString;

public class FalconSetVelocity extends CommandBase {
  /** Creates a new SetVelocity. */
  private FalconController motor;
  private FileLog log;
  private boolean fromShuffleboard;
  private double velocity;

  /**
   * Sets the motor to a specific velocity using the PID controller.
   * The velocity is read from Shuffleboard, in RPM
   * @param motor motor subsystem
   * @param log log file
   */
  public FalconSetVelocity(FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    fromShuffleboard = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);

    SmartDashboard.putNumber(buildString(motor.getName(), " SetPoint RPM"), 0.0);
  }

  /**
   * Sets the motor to a specific velocity using the PID controller.
   * @param velocity in RPM
   * @param motor motor subsystem
   * @param log log file
   */
  public FalconSetVelocity(double velocity, FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    fromShuffleboard = false;
    this.velocity = velocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      velocity = SmartDashboard.getNumber(buildString(motor.getName(), " SetPoint RPM"), 0.0);
    } else {
      SmartDashboard.putNumber(buildString(motor.getName(), " SetPoint RPM"), velocity);
    }
    
    log.writeLog(false, motor.getName(), "Set RPM", "Set Velocity", velocity);
    motor.setMotorVelocity(velocity);
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
    if(Math.abs(motor.getVelocityPIDError()) < ShooterConstants.pidErrorTolerance) {
      log.writeLogEcho(false, motor.getName(), "Set RPM at speed", "Set Velocity", velocity, "Velocity", motor.getMotorVelocity());
      return true;
    }
      return false;
  }
}
