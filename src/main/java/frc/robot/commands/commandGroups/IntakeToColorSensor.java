// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToColorSensor extends SequentialCommandGroup {
  /** Creates a new IntakeToColorSensor. */
  public IntakeToColorSensor(Intake intake, Uptake uptake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetPercentOutput(0.25, 0.25, intake, log),
      new UptakeSetPercentOutput(0.25, 0, uptake, log)
    );
  }
}
