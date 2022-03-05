// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import org.ejml.equation.Sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallCountAddBall;
import frc.robot.commands.BallCountSubtractBall;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  /** Creates a new ShootBall. */
  public ShootBall(double velocity, BallColor ejectColor, Shooter shooter, Uptake uptake, Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new ConditionalCommand(
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new FeederSetPercentOutput(.25, feeder, log),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kShooter, log),
            new UptakeSortBall(ejectColor, uptake, feeder, log),
            new WaitCommand(2).perpetually().withInterrupt(feeder::isBallPresent),
            new ConditionalCommand(
              sequence(
                new WaitCommand(2),
                new FeederStop(feeder, log),
                new ShooterStop(shooter, log),
                new BallCountSubtractBall(BallLocation.kShooter, log)
              ), 
              sequence(
                new FeederStop(feeder, log),
                new ShooterStop(shooter, log),
                new UptakeStop(uptake, log)
              ), 
              () -> feeder.isBallPresent()
              )
          ), 
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new FeederSetPercentOutput(0.25, feeder, log),
            parallel(
              new BallCountSubtractBall(BallLocation.kFeeder, log),
              new BallCountAddBall(BallLocation.kShooter, log)
            ),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kShooter, log),
            new ShooterStop(shooter, log),
            new FeederStop(feeder, log)
          ),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 1
        ),
        new ConditionalCommand(
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new FeederSetPercentOutput(0.25, feeder, log),
            new UptakeSortBall(ejectColor, uptake, feeder, log),
            new WaitCommand(2).perpetually().withInterrupt(feeder::isBallPresent),
            new ConditionalCommand(
              sequence(
                parallel(
                new BallCountAddBall(BallLocation.kShooter, log),
                new BallCountSubtractBall(BallLocation.kFeeder, log)
                ),
                new WaitCommand(2),
                new BallCountSubtractBall(BallLocation.kShooter, log)
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
