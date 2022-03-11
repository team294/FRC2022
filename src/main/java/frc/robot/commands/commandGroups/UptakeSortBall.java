package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.LogEnableFastLogging;
import frc.robot.commands.UptakeEjectBall;
import frc.robot.commands.UptakeFeedBall;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeSortBall extends SequentialCommandGroup {

  /**
   * Sequence to handle a ball once it hits the color sensor in the uptake
   * Ejects the ball if it matches the ejectColor
   * 
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param log logger
   */
  public UptakeSortBall(Uptake uptake, Feeder feeder, FileLog log) {

    addCommands(
      new FileLogWrite(true, false, "UptakeSortBall", "start sequence", log, "ejectColor",uptake.getEjectColor()),
      new LogEnableFastLogging(true, uptake, log),
      new WaitCommand(0.1),
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
      () -> uptake.getBallColor().equals(uptake.getEjectColor())
      ),
      new LogEnableFastLogging(false, uptake, log),
      new WaitCommand(0.5),
      // turn off uptake if we have balls in feeder and uptake, otherwise turn it on
      new ConditionalCommand(
          new UptakeSetPercentOutput(0, 0, uptake, log),
          new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log), 
        () -> feeder.isBallPresent() && uptake.isBallPresent()),
      new FileLogWrite(true, false, "UptakeSortBall", "end sequence", log)
    );
  }
}


