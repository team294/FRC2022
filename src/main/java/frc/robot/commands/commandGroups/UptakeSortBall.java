package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BallColor;
import frc.robot.commands.UptakeEjectBall;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.UptakeToFeeder;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeSortBall extends SequentialCommandGroup {

  /**
   * Sequence to handle a ball once it hits the color sensor in the uptake
   * Ejects the ball if it matches the ejectColor
   * 
   * @param ejectColor Color to eject
   * @param uptake uptake subsystem
   * @param log logger
   */
  public UptakeSortBall(BallColor ejectColor, Uptake uptake, Feeder feeder, FileLog log) {

    addCommands(
      new ConditionalCommand(
        // if it is the wrong color, eject the ball
        sequence(
          new UptakeEjectBall(uptake, log).withTimeout(1)
        ),
        // if it is the right color then check if there is room in the feeder
        new ConditionalCommand(
          // if there is nothing in the feeder then feed it
          sequence(
            new UptakeToFeeder(uptake, feeder, log).withTimeout(1)
          ),
          // if there is something in the feeder do nothing
          new WaitCommand(0.2),
          () -> !feeder.isBallPresent()
        ),
      () -> uptake.getBallColor() == ejectColor
      )
    );
  }
}


