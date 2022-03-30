package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Shoot sequence triggers that action of shooting all the balls by running the feeder and uptake.
   * The aiming of the turret and the speed of the shooter should be set before this sequence is run.
   * 
   * @param intake intake subsystem
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param shooter shooter subsystem
   * @param log
   */
  public ShootSequence(Intake intake, Uptake uptake, Feeder feeder, Shooter shooter, FileLog log) {
    addCommands(
      new ConditionalCommand( 
        // only shoot if the shooter is not at idle
        sequence(
          new FileLogWrite(true, false, "ShootSequence", "shooting", log),
          new ShooterSetVelocity(InputMode.kLastSetSpeed, shooter, log).withTimeout(1),    // Wait for shooter to be at speed
          new FeederSetPercentOutput(FeederConstants.onPct, feeder, log),         // turn on feeder to send first ball to shooter
          new WaitCommand(2).withInterrupt(() -> !feeder.isBallPresent()),        // turn off feeder when ball clears feeder
          new FeederSetPercentOutput(0, feeder, log),         // turn off feeder
          new WaitCommand(0.2),                           // Give first ball a little time to clear feeder

          new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPct, intake, log), // turn on transfer wheels to clear jams
          new UptakeSetPercentOutput(UptakeConstants.onPct, false, uptake, log),  // make sure uptake is running to send second ball to feeder
          new ShooterSetVelocity(InputMode.kLastSetSpeed, shooter, log).withTimeout(1),    // Wait for shooter to be at speed
          new FeederSetPercentOutput(FeederConstants.onPct, feeder, log),         // turn on feeder to send second ball to shooter

          new WaitCommand(1.0),                                 // wait for second ball to shoot 
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
