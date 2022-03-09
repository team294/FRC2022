// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.FileLog;

public class XboxRumble extends CommandBase {
  private XboxController xboxController;
  private double percentRumble;
  private double targetTime;
  private FileLog log;
  private int seconds;
  /** Creates a new XboxRumble. */
  /**
   * Rumbles the Xbox Controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   * @param seconds time in seconds to rumble
   * @param xboxController xbox controller
   * @param log logger
   */
  public XboxRumble(double percentRumble, int seconds, XboxController xboxController, FileLog log) {
    this.xboxController = xboxController;
    this.percentRumble = percentRumble;
    this.log = log;
    this.seconds = seconds;
    targetTime = System.currentTimeMillis() + (seconds * 1000);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new FileLogWrite(false, false, "XboxRumble", "Starting rumble for " + seconds + "seconds", log);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() > targetTime){
      return true;
    }else return false;
  }
}