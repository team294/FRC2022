// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;


public class FalconControlWithJoystick extends CommandBase {

  private final FalconController motor;
  private final Joystick joystick;
  private final FileLog log;


  /**
   * Sets the motor speed from the joystick.  Command does not stop until cancelled.
   * @param motor motor subsystem
   * @param log logfile
   */
  public FalconControlWithJoystick(Joystick joystick, FalconController motor, FileLog log) {
    this.joystick = joystick;
    this.motor = motor;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, motor.getName(), "Control with Joystick Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double percent = -joystick.getY();
    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      log.writeLog(false, motor.getName(), "Control with Joystick", "Value", percent);
    }

    motor.setMotorPercentOutput(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, motor.getName(), "Control with Joystick Stop");
    motor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
