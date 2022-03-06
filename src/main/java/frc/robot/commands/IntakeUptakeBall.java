// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class IntakeUptakeBall extends CommandBase {
  private Intake intake;
  private Uptake uptake;
  private FileLog log;
  private BallColor teamColor;
  /** Creates a new IntakeUptakeBall. */
  public IntakeUptakeBall(BallColor teamColor, Intake intake, Uptake uptake, FileLog log) {
    this.intake = intake;
    this.uptake = uptake;
    this.log = log;
    this.teamColor = teamColor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(BallCount.getTotalBallCount() == 2) {
      intake.stopMotor();
      uptake.stopMotor();
    } else {
      intake.setMotorPercentOutput(0.25);
      uptake.setUptakePercentOutput(0.25);
    }

    if(uptake.colorSensor.isBallPresent()){
      BallCount.setBallCount(1, BallLocation.kUptake, log);
      if(uptake.colorSensor.getBallColor() == teamColor){
        if(BallCount.getBallCount(BallLocation.kFeeder) == 0){
          uptake.setEjectPercentOutput(0.25);
          BallCount.setBallCount(1, BallLocation.kFeeder, log);
          BallCount.setBallCount(0, BallLocation.kUptake, log);
        }
      }
      else{
        uptake.setEjectPercentOutput(-0.25);
        BallCount.setBallCount(1, BallLocation.kEject, log);
        BallCount.setBallCount(0, BallLocation.kUptake, log);
      }
    }
    else{
      uptake.setEjectPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
