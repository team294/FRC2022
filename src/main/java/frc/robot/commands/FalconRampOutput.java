// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class FalconRampOutput extends CommandBase {

  private final FalconController motor;
  private final FileLog log;
  private double startPercent = 0.0, stopPercent = 0.5;
  private double stepPercent = 10.0;
  private double percentOut = 0.0;

  /**
   * Ramps the motor percent from startPercent to stopPercent over rampTimeSec seconds.
   * After the ramp, the motor speed is held at stopPercent until the command
   * is cancelled.
   * @param startPercent motor percent, -1.0 to 1.0
   * @param stopPercent motor percent, -1.0 to 1.0
   * @param rampTimeSec ramp time, in seconds
   * @param motor motor subsystem
   * @param log logfile
   */
  public FalconRampOutput(double startPercent, double stopPercent, double rampTimeSec,
      FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    this.startPercent = startPercent;
    this.stopPercent = stopPercent;
    stepPercent = (stopPercent - startPercent)*0.020/rampTimeSec;   // Execute is called every 20msec

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percentOut = startPercent;
    motor.enableFastLogging(true);

    log.writeLog(false, motor.getName(), "RampOutput Start", "Start Percent", startPercent, 
        "Stop Percent", stopPercent, "stepPercent", stepPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber(buildString(motor.getName(), " Percent"), percentOut);
    motor.setMotorPercentOutput(percentOut);

    if ( ( (stepPercent>0) && (percentOut<stopPercent) ) ||
         ( (stepPercent<0) && (percentOut>stopPercent) ) ) {
      percentOut += stepPercent;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.stopMotor();

    log.writeLog(false, motor.getName(), "RampOutput Finish", "Final Percent", percentOut);
    motor.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
