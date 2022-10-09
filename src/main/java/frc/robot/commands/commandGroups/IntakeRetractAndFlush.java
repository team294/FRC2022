// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractAndFlush extends SequentialCommandGroup {
  private DigitalInput ballSensorTop = new DigitalInput(Ports.DIOUptakeTop) ; // Senses when a ball is between the uptake and the feeder
  private DigitalInput ballSensorMid = new DigitalInput(Ports.DIOUptakeMid) ; // Senses when a ball is at the color sensor
  private DigitalInput ballSensorFront = new DigitalInput(Ports.DIOUptakeFront) ; // Senses when a ball is entering the uptake
  /** Creates a new IntakeRetractAndFlush. */
  public IntakeRetractAndFlush(IntakeFront intakeFront, Uptake uptake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeRetractAndTurnOffMotors(intakeFront, uptake, log),

      new ConditionalCommand(
        sequence(
          new FlushSequence(intakeFront, log, 0.5)
        )
        ,
          sequence(), 
          () -> (!(ballSensorTop.get() && ballSensorFront.get()) && !(ballSensorFront.get() && ballSensorMid.get()) && !(ballSensorMid.get() && ballSensorTop.get()))
          
        )

    );
  }
}
