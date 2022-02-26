// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UptakeSortBall extends SequentialCommandGroup {

  /**
   * Uptakes a ball to the feeder.  Ejects the ball if it matches the ejectColor.
   * @param ejectColor Color to eject
   * @param uptake
   * @param log
   */
  public UptakeSortBall(BallColor ejectColor, Uptake uptake, Feeder feeder, FileLog log) {

    addCommands(
      // If there is not a ball in the uptake, then load the ball into the uptake
      new ConditionalCommand(
        new WaitCommand(0.02),
        sequence(
          new UptakeSetPercentOutput(0.25, false, uptake, log)
          .perpetually().withInterrupt(uptake.colorSensor::isBallPresent), 
          new BallCountAddBall(BallLocation.kUptake, log),
          new BallCountSubtractBall(BallLocation.kIntake, log)
        ),
        uptake.colorSensor::isBallPresent
      ),

      // Depending on color, either load or eject the ball
      new ConditionalCommand(
        // Eject ball
        sequence(
          new UptakeSetPercentOutput(0.25, true, uptake, log),
          new WaitCommand(2).perpetually().withInterrupt(uptake::isBallPresent), 
          new WaitCommand(.5),
          new BallCountSubtractBall(BallLocation.kUptake, log),
          new UptakeStop(uptake, log)
        ),
        // Load ball to feeder
        sequence(
          new UptakeSetPercentOutput(0.25, false, uptake, log), 
          new WaitCommand(2).perpetually().withInterrupt(feeder::isBallPresent), 
          new UptakeStop(uptake, log),
          parallel(
            new BallCountAddBall(BallLocation.kFeeder, log),
            new BallCountSubtractBall(BallLocation.kUptake, log)
          )  
        ),
        () -> uptake.colorSensor.getBallColor() == ejectColor
      ),

      // Clear the ball out of the uptake
      new WaitCommand(2.0).withInterrupt( () -> !uptake.colorSensor.isBallPresent()),
      new WaitCommand(2),
      new UptakeStop(uptake, log)
    );
  }
}
