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
 * @param useVision true if vision should be used for determining velocity
 * @param velocity target rpm of the shooter if not using vision or if vision can't see target
 * @param vision vision subsystem to use for distance. Set to null when not using vision.
 * @param shooter shooter subsystem
 * @param log log subsystem
 */
public class ShootSetup extends SequentialCommandGroup {

  public ShootSetup(boolean useVision, double velocity, PiVisionHub vision, Shooter shooter, FileLog log) {

    addCommands(
      sequence(
        new FileLogWrite(false, false, "ShootSetup", "Setup", log, "Velocity", velocity, "useVision", vision == null ? "no":"yes" ),
        new ConditionalCommand(
          // get velocity from vision if vision is available and can see the target
          sequence (
            new FileLogWrite(false, false, "ShootSetup", "SetupWithVision", log, "Velocity", velocity, "Vision", vision == null ? "0":vision.getDistance()),
            new ShooterSetVelocity(InputMode.kSpeedRPM, (vision == null) ? velocity : shooter.distanceFromTargetToRPM(vision.getDistance()), shooter, log)
          ),
          // use velocity without vision
          sequence (
            new FileLogWrite(false, false, "ShootSetup", "SetupWithVelocity", log, "velocity", velocity),
            new ShooterSetVelocity(InputMode.kSpeedRPM, velocity, shooter, log)
          ),
          () -> (useVision && vision != null)
        ),
        new FileLogWrite(false, false, "ShootSetup", "End", log, "Velocity", velocity, "useVision", useVision,"visionDistance", vision == null ? vision.getDistance():"null vision"))
      );
  }
}
