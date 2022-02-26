// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconController;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.buildString;
import static frc.robot.Constants.*;

public class FalconSetPIDSV extends CommandBase {

  private FileLog log;
  private FalconController motor;
  private boolean fromShuffleboard;
  private double P, I, D, S, V;

  /**
   * Sets PIDSV parameters for motor velocity control from Shuffleboard.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / FalconConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * 
   * <p> P = Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * <p> I = Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * <p> D = Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * <p> S = Feedforward offset, in percent output (-1 to +1)
   * <p> V = Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   * @param motor Motor subsystem to set PIDF parameters
   * @param log File for logging
   */
  public FalconSetPIDSV(FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    this.fromShuffleboard = true;
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber(buildString(motor.getName(), " P"), FalconConstants.kP);
    SmartDashboard.putNumber(buildString(motor.getName(), " I"), FalconConstants.kI);
    SmartDashboard.putNumber(buildString(motor.getName(), " D"), FalconConstants.kD);
    // SmartDashboard.putNumber(buildString(motor.getName(), " FF"), FalconConstants.kFF);    
    SmartDashboard.putNumber(buildString(motor.getName(), " S"), FalconConstants.kS);
    SmartDashboard.putNumber(buildString(motor.getName(), " V"), FalconConstants.kV);
  }

  /**
   * Sets PIDSV parameters for motor velocity control.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / FalconConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * @param P Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * @param I Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * @param D Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * @param S Feedforward offset, in percent output (-1 to +1)
   * @param V Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   * @param motor Motor subsystem to set PIDF parameters
   * @param log File for logging
   */
  public FalconSetPIDSV(double P, double I, double D, double S, double V, FalconController motor, FileLog log) {
    this.motor = motor;
    this.log = log;
    this.fromShuffleboard = false;
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      // Get PID coefficients from SmartDashboard
      P = SmartDashboard.getNumber(buildString(motor.getName(), " P"), FalconConstants.kP);
      I = SmartDashboard.getNumber(buildString(motor.getName(), " I"), FalconConstants.kI);
      D = SmartDashboard.getNumber(buildString(motor.getName(), " D"), FalconConstants.kD);
      // F = SmartDashboard.getNumber(buildString(motor.getName(), " FF"), FalconConstants.kFF);
      S = SmartDashboard.getNumber(buildString(motor.getName(), " S"), FalconConstants.kS);
      V = SmartDashboard.getNumber(buildString(motor.getName(), " V"), FalconConstants.kV);
    } else {
      SmartDashboard.putNumber(buildString(motor.getName(), " P"), P);
      SmartDashboard.putNumber(buildString(motor.getName(), " I"), I);
      SmartDashboard.putNumber(buildString(motor.getName(), " D"), D);
      // SmartDashboard.putNumber(buildString(motor.getName(), " FF"), F);
      SmartDashboard.putNumber(buildString(motor.getName(), " S"), S);
      SmartDashboard.putNumber(buildString(motor.getName(), " V"), V);
    }

    log.writeLog(false, motor.getName(), "Set PIDF", "P", P, "I", I, "D", D, "S", S, "V", V);
    motor.setPIDSV(P, I, D, S, V);
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
