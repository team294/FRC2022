// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  /** Creates a new ShootBall. */
  public ShootBall(double velocity, Shooter shooter, Uptake uptake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new ConditionalCommand(
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new WaitCommand(2),
            new ShooterStop(shooter, log),
            new BallCountSubtractBall(BallLocation.kShooter, log)
          ), 
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kShooter, log),
            new UptakeSetPercentOutput(0.25, false, uptake, log),
            new BallCountSubtractBall(BallLocation.kUptake, log),
            new BallCountAddBall(BallLocation.kShooter, log),
            new WaitCommand(2),
            new ShooterStop(shooter, log),
            new UptakeStop(uptake, log)
          ),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 0
        ),
        new ConditionalCommand(
          sequence(
            new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new UptakeSetPercentOutput(0.25, true, uptake, log),
            new BallCountSubtractBall(BallLocation.kUptake, log)
          ), 
          new WaitCommand(.2),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 1
        ),
        () -> BallCount.getBallCount(BallLocation.kUptake) == 1
      )  
    );
  }
}
