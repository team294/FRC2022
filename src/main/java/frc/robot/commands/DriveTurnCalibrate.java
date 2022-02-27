/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveTurnCalibrate extends CommandBase {
  private DriveTrain driveTrain;
  private FileLog log;
  private double maxPctOut, rampTime, rampRate, direction;
  private double motorPctOut, gyroStart;
  private final Timer timer = new Timer();

  /**
   * Creates a new DriveTurnCalibrate.  Turns in place and logs the driveTelemetry.
    * @param maxPctOut  Absolute value 0 to 1.  Power during ramp is clamped to this value.
    * @param rampTime Time in seconds for ramp
    * @param rampRate Ramp rate in pctOut/second 
    * @param turnLeft True = turn left, false = turn right
    * @param driveTrain
    * @param log
    */
  public DriveTurnCalibrate(double maxPctOut, double rampTime, double rampRate, boolean turnLeft, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.maxPctOut = Math.abs(maxPctOut);
    this.driveTrain = driveTrain;
    this.log = log;
    this.rampTime = Math.abs(rampTime);
    this.rampRate = Math.abs(rampRate);
    direction = turnLeft ? 1.0 : -1.0;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    motorPctOut = 0;
    gyroStart = driveTrain.getGyroRotation();
    driveTrain.setOpenLoopRampLimit(false);

    log.writeLog(false, "DriveTurnCalibrate", "init", "maxPctOut", maxPctOut, "rampTime", rampTime, "rampRate", rampRate, "direction", direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();

    log.writeLog(false, "DriveTurnCalibrate", "execute", "time", curTime, 
      "pctOut", motorPctOut,
      "VbusLeft", driveTrain.getLeftBusVoltage(), "VbusRight", driveTrain.getRightBusVoltage(),
      "Vleft", driveTrain.getLeftOutputVoltage(), "VRight", driveTrain.getRightOutputVoltage(),
      "Ileft", driveTrain.getLeftStatorCurrent(), "IRight", driveTrain.getRightStatorCurrent(),
      "gyro", driveTrain.normalizeAngle(driveTrain.getGyroRotation()-gyroStart),
      "gyro vel", driveTrain.getAngularVelocity(),
      "ang vel from wheels", driveTrain.getAngularVelocityFromWheels(),
      "Lpos", driveTrain.getLeftEncoderInches(), "Lvel", driveTrain.getLeftEncoderVelocity(),
      "Rpos", driveTrain.getRightEncoderInches(), "Rvel", driveTrain.getRightEncoderVelocity()
    );

    motorPctOut = MathUtil.clamp( curTime*rampRate, -maxPctOut, maxPctOut);

    driveTrain.setLeftMotorOutput(-direction*motorPctOut);
    driveTrain.setRightMotorOutput(direction*motorPctOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrain.setOpenLoopRampLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(rampTime);
  }
}
