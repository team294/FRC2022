// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

public class StopAllMotors extends CommandBase {
  /** Creates a new StopAllMotors. */
  Feeder feeder;
  Uptake uptake;
  Shooter shooter;
  Intake intake;
  public StopAllMotors(Feeder feeder, Uptake uptake, Shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = feeder;
    this.shooter = shooter;
    this.uptake = uptake;
    this.intake = intake;
    addRequirements(intake);
    addRequirements(feeder);
    addRequirements(shooter);
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.stopMotor();
    shooter.stopMotor();
    uptake.stopMotor();
    intake.stopMotor();
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
