package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BallColor;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.UptakeEjectBall;
import frc.robot.commands.UptakeFeedBall;
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
      new FileLogWrite(true, false, "UptakeSortBall", "start sequence", log),
      new ConditionalCommand(
        // if it is the wrong color, eject the ball
        sequence(
          new FileLogWrite(true, false, "UptakeSortBall", "eject", log),
          new UptakeEjectBall(uptake, log).withTimeout(1)
        ),
        // if it is the right color then check if there is room in the feeder
        new ConditionalCommand(
          // if there is nothing in the feeder then feed it
          sequence(
            new FileLogWrite(true, false, "UptakeSortBall", "feed", log),
            new UptakeFeedBall(uptake, feeder, log).withTimeout(1)
            //new UptakeToFeeder(uptake, feeder, log).withTimeout(1)
          ),
          // if there is something in the feeder do nothing
          new FileLogWrite(true, false, "UptakeSortBall", "hold", log),
          () -> !feeder.isBallPresent()
        ),
      () -> uptake.getBallColor().equals(ejectColor)
      ),
      new FileLogWrite(true, false, "UptakeSortBall", "end sequence", log)
    );
  }
}


