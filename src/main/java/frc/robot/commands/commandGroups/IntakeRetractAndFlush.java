// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractAndFlush extends SequentialCommandGroup {

  /** Creates a new IntakeRetractAndFlush. */
  public IntakeRetractAndFlush(IntakeFront intakeFront, Uptake uptake, Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeRetractAndTurnOffMotors(intakeFront, uptake, log),

      new ConditionalCommand(
          new FlushSequence(intakeFront, log, 0.5),
          new WaitCommand(0.01), 
          () -> !((uptake.isBallGoingToFeeder() && uptake.isBallAtColorSensor()) || (uptake.isBallGoingToFeeder() && feeder.isBallPresent()) || (uptake.isBallAtColorSensor() && feeder.isBallPresent()))
          
        )

    );
  }
}
