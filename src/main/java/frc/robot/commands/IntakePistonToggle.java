package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;


public class IntakePistonToggle extends CommandBase {
  private Intake intake;
  private FileLog log;

  /**
   * Toggles the intake between deployed and retracted 
   * 
   * @param intake intake subsystem
   * @param log
   */
  public IntakePistonToggle(Intake intake, FileLog log) {
    this.intake = intake;
    this.log = log;
    addRequirements(intake);
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, intake.getName(), "Toggle");

    // turn off/on intake before closing/opening intake
    if (intake.getPistonExtended()) {
      intake.setMotorPercentOutput(0);
      intake.setPistonExtended(false);
    } else {
      intake.setPistonExtended(true);
      intake.setMotorPercentOutput(IntakeConstants.intakeDefaultPercent);
    }
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
