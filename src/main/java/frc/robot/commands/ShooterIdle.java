package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterIdle extends CommandBase {
  
  private Shooter shooter;
  private final FileLog log;

  /**
   * Idles the shooter motor
   * @param shooter shooter subsystem
   * @param log logfile
   */
  public ShooterIdle(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, shooter.getName(), "Idle");
    shooter.setMotorVelocity(ShooterConstants.shooterDefaultRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
