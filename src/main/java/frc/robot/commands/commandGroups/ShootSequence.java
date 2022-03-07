package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetBallCount;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.UptakeStop;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FeederStop;
import frc.robot.Constants.BallLocation;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Shoot sequence triggers that action of shooting all the balls by running the feeder and uptake.
   * The aiming of the turret and the speed of the shooter should be set before this sequence is run.
   * 
   * @param shooter shooter subsystem
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param log
   */
  public ShootSequence(Shooter shooter, Uptake uptake, Feeder feeder, FileLog log) {
    addCommands(
      new ConditionalCommand( 
        // only shoot if the shooter is not at idle
        sequence(
          new FeederSetPercentOutput(feeder, log),              // turn on feeder to send first ball to shooter
          new WaitCommand(1),                                   // wait for ball to shoot
          new UptakeSetPercentOutput(0.25, false, uptake, log), // turn on uptake to send second ball to shooter
          new WaitCommand(2),                                   // wait for balls to shoot
          new SetBallCount(0, BallLocation.kFeeder, log),       // zero feeder ball count
          new SetBallCount(0, BallLocation.kUptake, log),       // zero update ball count
          new FeederStop(feeder, log),                          // turn off the feeder
          new UptakeStop(uptake, log)                           // turn off the uptake
        ),
        new WaitCommand(0.2),
        () -> shooter.getMotorVelocity() > ShooterConstants.shooterDefaultRPM
      )
    );
  }
}
