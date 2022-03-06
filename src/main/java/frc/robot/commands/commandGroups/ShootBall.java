package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallCountAddBall;
import frc.robot.commands.BallCountSubtractBall;
import frc.robot.commands.ShooterIdle;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.UptakeStop;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class ShootBall extends SequentialCommandGroup {

  /**
   * Shoot sequence for balls
   * 
   * @param shooter shooter subsystem
   * @param uptake update subsystem
   * @param log file log
   * 
  */
  public ShootBall(Shooter shooter, Uptake uptake, FileLog log) {

    addCommands(
      new ConditionalCommand(

        // if there is a ball in the uptake
        new ConditionalCommand(

          // if there are no balls in the uptake then shoot then stop the shooter
          sequence(
            //new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new WaitCommand(2),
            new ShooterIdle(shooter, log),
            new BallCountSubtractBall(BallLocation.kShooter, log)
          ), 
          
          // if there are balls in the uptake then shoot, run the uptake to move the next ball into the shooter
          sequence(
            //new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
            new WaitCommand(2),
            new BallCountSubtractBall(BallLocation.kShooter, log),
            new UptakeSetPercentOutput(0.25, false, uptake, log),
            new BallCountSubtractBall(BallLocation.kUptake, log),
            new BallCountAddBall(BallLocation.kShooter, log),
            new WaitCommand(2),
            new ShooterIdle(shooter, log),
            new UptakeStop(uptake, log)
          ),  
          () -> BallCount.getBallCount(BallLocation.kUptake) == 0
        ),

        // if there was not a ball in the uptake
        new ConditionalCommand(
          // if there is a ball in the uptake then move it to the shooter
          sequence(
            //new ShooterSetVelocity(InputMode.kDistFeet, velocity, shooter, log),
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
