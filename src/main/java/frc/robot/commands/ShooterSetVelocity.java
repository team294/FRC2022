
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterSetVelocity extends CommandBase {

  public enum InputMode {
    kSpeedRPM,
    kDistInch; // TODO 9 feet for (3400) medium shot, short shot (3100) 7 feet
  }
  
  private Shooter shooter;
  private PiVisionHub piVisionHub;
  private FileLog log;
  private boolean fromShuffleboard;
  private double velocity;
  private InputMode mode;

  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * The velocity is read from Shuffleboard, depending on the mode:
   * InputMode.kSpeedRPM = "Shooter SetPoint RPM"
   * InputMode.kDistFeed = "Shooter Distnace Feet"
   * @param mode InputMode.kSpeedRPM or InputMode.kDistInch
   * @param shooter shooter subsystem
   * @param log log file
   */
  public ShooterSetVelocity(InputMode mode, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.mode = mode;
    this.fromShuffleboard = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    if (mode == InputMode.kDistInch) {
      SmartDashboard.putNumber("Shooter Distance Feet", 5.0);
    } else {
      SmartDashboard.putNumber("Shooter SetPoint RPM", 2000.0);
    }
  }

  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * @param mode InputMode.kSpeedRPM or InputMode.kDistInch -- Defines units for "value" parameter
   * @param value setpoint speed (in RMP) or distance (in feet)
   * @param shooter shooter subsystem
   * @param log log file
   */
  public ShooterSetVelocity(InputMode mode, double value, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.mode = mode;
    this.fromShuffleboard = false;
    
    log.writeLog(false, "Shooter", "SetVelocity", "mode", mode, "value", value,"fromShuffleboard", fromShuffleboard);
    if (mode == InputMode.kDistInch) {
      velocity = 0;
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from distance", velocity);
    } else {
      velocity = value;
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from rpm", velocity);
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * @param mode InputMode.kSpeedRPM or InputMode.kDistInch -- Defines units for "value" parameter
   * @param value setpoint speed (in RMP) or distance (in feet)
   * @param shooter shooter subsystem
   * @param log log file
   */
  public ShooterSetVelocity(InputMode mode, double value, Shooter shooter, PiVisionHub pivisionhub, FileLog log) {
    this.shooter = shooter;
    this.piVisionHub = pivisionhub;
    this.log = log;
    this.mode = mode;
    this.fromShuffleboard = false;
    
    log.writeLog(false, "Shooter", "SetVelocity", "mode", mode, "value", value,"fromShuffleboard", fromShuffleboard);
    if (mode == InputMode.kDistInch) {
      velocity = 0;
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from distance", velocity);
    } else {
      velocity = value;
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from rpm", velocity);
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivisionhub);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      if (mode==InputMode.kDistInch) {
        double dist = SmartDashboard.getNumber("Shooter Distance Feet", 5.0);
        velocity = shooter.distanceFromTargetToRPM(dist);
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromShuffleboardDistance", velocity);
        SmartDashboard.putNumber("Shoter SetPoint RPM", velocity);
      } else {
        velocity = SmartDashboard.getNumber("Shooter SetPoint RPM", 0.0);
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromShuffleboardRPM", velocity);
      }
    } else {
      if (mode == InputMode.kDistInch && piVisionHub != null) {
        velocity = shooter.distanceFromTargetToRPM(piVisionHub.getDistance());
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromVision", velocity);
      } 
      SmartDashboard.putNumber("Shooter SetPoint RPM", velocity);
    }
    
    if(velocity >= 2000){
      shooter.enableFastLogging(true);
    }
    log.writeLog(false, "Shooter SetVelocity", "Initialize", "Set Velocity", velocity);
    shooter.setMotorVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getVelocityPIDError()) < ShooterConstants.pidErrorTolerance) {
      log.writeLogEcho(false, "Shooter SetVelocity", "Set RPM at speed", "Set Velocity", velocity, "Velocity", shooter.getMotorVelocity());
      return true;
    }
      return false;
  }
}
