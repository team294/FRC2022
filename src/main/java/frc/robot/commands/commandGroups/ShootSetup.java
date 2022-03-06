package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

/**
 * Set shooter velocity with vision or set point
 * @param velocity target rpm of the shooter if not using vision or if vision can't see target
 * @param vision vision subsystem to use for distance. Set to null when not using vision.
 * @param shooter shooter subsystem
 * @param log log subsystem
 */
public class ShootSetup extends SequentialCommandGroup {

  public ShootSetup(double velocity, PiVisionHub vision, Shooter shooter, FileLog log) {

    addCommands(
          sequence(
            new FileLogWrite(false, false, "ShootSetup", "Setup", log, "Velocity", velocity, "useVision", vision == null ? "no":"yes" ),
            new ConditionalCommand(
              new ShooterSetVelocity(InputMode.kSpeedRPM, velocity, shooter, log),
              new ShooterSetVelocity(InputMode.kDistFeet, vision.getDistance(), shooter, log),
              () -> (vision != null && vision.getDistance() != 0)
            )
            
          )

      
    );
  }
}
