package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Shoot sequence triggers that action of shooting all the balls by running the feeder and uptake.
   * The aiming of the turret and the speed of the shooter should be set before this sequence is run.
   * 
   * @param intake intake subsystem
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param log
   */
  public ShootSequence(Intake intake, Uptake uptake, Feeder feeder, FileLog log) {
    addCommands(
      new ConditionalCommand( 
        // only shoot if the shooter is not at idle
        sequence(
          new FileLogWrite(true, false, "ShootSequence", "shooting", log),
          new FeederSetPercentOutput(FeederConstants.onPct, feeder, log),         // turn on feeder to send first ball to shooter
          new WaitCommand(1), 
          new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPct, intake, log), // turn on transfer wheels for jams
          new UptakeSetPercentOutput(UptakeConstants.onPct, false, uptake, log),  // make sure uptake is running to send second ball to shooter
          //new WaitCommand(1),                                   // wait for balls to shoot
          new FeederSetPercentOutput(0, feeder, log),           // turn off the feeder
          new IntakeToColorSensor(intake, uptake, log)          // turn on intake
        ),
        new FileLogWrite(true, false, "ShootSequence", "Shooter not ready", log),
        () -> true
        //shooter.getMotorVelocity() > ShooterConstants.shooterDefaultRPM
      ),
      new FileLogWrite(true, false, "ShootSequence", "end", log)
    );
  }
}
