package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class IntakeToColorSensor extends SequentialCommandGroup {

/**
 * Turn on intake and uptake to intake a ball up to the color sensor and hold
 * 
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param log file logger
 */
public IntakeToColorSensor(Intake intake, Uptake uptake, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "IntakeToColorSensor", "starting", log),
      new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPct, intake, log),
      new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)
    );
  }
}
