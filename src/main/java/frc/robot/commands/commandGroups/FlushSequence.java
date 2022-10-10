// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlushSequence extends SequentialCommandGroup {
  /** Creates a new FlushSequence. */
  public FlushSequence(Intake intake, Uptake uptake, FileLog log, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetPercentOutput(-IntakeConstants.onPct, IntakeConstants.onPct, intake, log), // turn on transfer wheels to clear jams, intake in reverse to clear jams (F7)
      new UptakeSetPercentOutput(UptakeConstants.onPct, false, uptake, log),
      new WaitCommand(time),
      new IntakeSetPercentOutput(0, 0, intake, log)


    );
  }
}
