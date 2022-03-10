package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TargetType;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakePistonSetPosition;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class AutoShootTaxi extends SequentialCommandGroup {

/**
 * Shoot preloaded ball and taxi out of the tarmac
 * 
 * @param waitTime seconds to wait before starting
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoShootTaxi(double waitTime, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, LimeLight limeLight, FileLog log) {
    addCommands(
      new WaitCommand(waitTime),                                        // delay from shuffleboard

      new DriveZeroGyro(0, driveTrain, log),      
      new FileLogWrite(false, false, "AutoShootTaxi", "starting", log),
      new ShooterSetVelocity(InputMode.kSpeedRPM, 3300, shooter, log),  // turn on the shooter
      new FeederSetPercentOutput(0.3, feeder, log),                     // turn on feeder to send first ball to shooter
      new WaitCommand(0.5),                                             // wait for ball to shoot
      new UptakeSetPercentOutput(0.3, false, uptake, log),              // make sure uptake is running just in case ball is jammed
      new WaitCommand(0.5),                                             // wait for ball to shoot
      new FeederSetPercentOutput(0, feeder, log),                       // turn off feeder
      new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log),  // turn off shooter

      // drive out backwards 
      new DriveStraight(-1.0, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log).withTimeout(2),

      // deploy intake so we are ready to go in teleop
      new IntakePistonSetPosition(true, intake, log),

      new IntakeToColorSensor(intake, uptake, log)

    );
  }
}
