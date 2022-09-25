// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakePistonSetPosition;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractAndTurnOffMotors extends SequentialCommandGroup {
  /** Creates a new IntakeRetract. */
  public IntakeRetractAndTurnOffMotors(Intake intake, Uptake uptake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakePistonSetPosition(false, intake, log),
      new WaitCommand(0.5),
      new IntakeSetPercentOutput(0, intake, log),
      new UptakeSetPercentOutput(0, 0, uptake, log)
    );
  }
}
