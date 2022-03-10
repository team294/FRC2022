package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;

public class IntakeUptakeBall extends CommandBase {
  private Intake intake;
  private Uptake uptake;
  private Feeder feeder;
  private FileLog log;
  private BallColor teamColor;



  /**
   * 
   * 
   * @param teamColor our team color
   * @param intake intake subsystem
   * @param uptake uptake subsystem
   * @param log
   */
  public IntakeUptakeBall(BallColor teamColor, Intake intake, Uptake uptake, Feeder feeder, FileLog log) {
    this.intake = intake;
    this.uptake = uptake;
    this.feeder = feeder;
    this.log = log;
    this.teamColor = teamColor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(uptake);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(uptake.isBallPresent()){
      BallCount.setBallCount(1, BallLocation.kUptake, log);
      if(uptake.getBallColor() == teamColor){
        if(BallCount.getBallCount(BallLocation.kFeeder) == 0){
          // send ball to feeder
          CommandScheduler.getInstance().schedule(new UptakeToFeeder(uptake, feeder, log));
        }
      }
      else{
        CommandScheduler.getInstance().schedule(new UptakeEjectBall(uptake, log));
      }
    }
    if(BallCount.getTotalBallCount() == 2) {
      CommandScheduler.getInstance().schedule(new IntakeStop(intake, log));
      CommandScheduler.getInstance().schedule(new UptakeStop(uptake, log));
    } else {
      CommandScheduler.getInstance().schedule(new IntakeSetPercentOutput(0.25, intake, log));
      uptake.setUptakePercentOutput(0.25);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
