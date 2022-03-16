package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FeederStop;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class StopAllMotors extends SequentialCommandGroup {

/**
 * Stop all motors
 * 
 * @param feeder feeder subsystem
 * @param shooter shooter subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param log file logger
 */
public StopAllMotors(Feeder feeder, Shooter shooter, Intake intake, Uptake uptake, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "StopAllMotors", "starting", log),
      new IntakeSetPercentOutput(0, 0, intake, log),
      new UptakeSetPercentOutput(0, 0, uptake, log),
      new FeederStop(feeder, log),
      new ShooterStop(shooter, log),
      new FileLogWrite(false, false, "StopAllMotors", "end", log)
    );
  }
}
