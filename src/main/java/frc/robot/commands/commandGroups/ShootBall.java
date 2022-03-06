package frc.robot.commands.commandGroups;

import org.ejml.equation.Sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallCountAddBall;
import frc.robot.commands.BallCountSubtractBall;
import frc.robot.commands.ShooterIdle;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.UptakeStop;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FeederStop;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.UptakeStop;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class ShootBall extends SequentialCommandGroup {
  /** Creates a new ShootBall. */
  public ShootBall(BallColor ejectColor, Shooter shooter, Uptake uptake, Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(

        // if there is a ball in the uptake
        new ConditionalCommand(

          // if there are no balls in the uptake then shoot then stop the shooter
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, shooter, log),
            new FeederSetPercentOutput(.25, feeder, log),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kFeeder, log),
            new UptakeSortBall(ejectColor, uptake, feeder, log),
            new WaitCommand(2).perpetually().withInterrupt(feeder::isBallPresent),
            new ConditionalCommand(
              sequence(
                new WaitCommand(2),
                new FeederStop(feeder, log),
                new ShooterStop(shooter, log),
                new BallCountSubtractBall(BallLocation.kFeeder, log)
              ), 
              sequence(
                new FeederStop(feeder, log),
                new ShooterStop(shooter, log),
                new UptakeStop(uptake, log)
              ), 
              () -> feeder.isBallPresent()
              )
          ), 
          
          // if there are balls in the uptake then shoot, run the uptake to move the next ball into the shooter
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, shooter, log),
            new FeederSetPercentOutput(0.25, feeder, log),
            parallel(
              new BallCountSubtractBall(BallLocation.kFeeder, log),
              new BallCountAddBall(BallLocation.kFeeder, log)
            ),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kFeeder, log),
            new ShooterStop(shooter, log),
            new FeederStop(feeder, log)
          ),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 1
        ),

        // if there was not a ball in the uptake
        new ConditionalCommand(
          // if there is a ball in the uptake then move it to the shooter
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, shooter, log),
            new FeederSetPercentOutput(0.25, feeder, log),
            new UptakeSortBall(ejectColor, uptake, feeder, log),
            new WaitCommand(2).perpetually().withInterrupt(feeder::isBallPresent),
            new ConditionalCommand(
              sequence(
                new WaitCommand(2),
                new BallCountSubtractBall(BallLocation.kFeeder, log)
              ),
              new WaitCommand(2),
              () -> feeder.isBallPresent()
          ),
            new ShooterStop(shooter, log),
            new UptakeStop(uptake, log),
            new FeederStop(feeder, log)
          ), 
          new WaitCommand(.2),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 1
        ),
        () -> feeder.isBallPresent()
      )  
    );
  }
}
