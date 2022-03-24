package frc.robot.commands.commandGroups;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.TargetType;
import frc.robot.commands.DriveFollowTrajectory;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveTurnGyro;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakePistonSetPosition;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.DriveFollowTrajectory.PIDType;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


public class AutoFourBall extends SequentialCommandGroup {

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
public AutoFourBall(double waitTime, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, TrajectoryCache trajectoryCache, PiVisionHub pivisionhub, LimeLight limeLight, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoFourBall", "starting", log),
      new WaitCommand(waitTime),                                        // delay from shuffleboard
      new DriveResetPose(0, 0, 0, driveTrain, log),     

      //TODO shoot two balls first, start looking at second ball
      new ShooterSetVelocity(InputMode.kSpeedRPM, AutoConstants.ballOneRPM, shooter, log),  // turn on the shooter
      new FeederSetPercentOutput(0.3, feeder, log),                     // turn on feeder to send first ball to shooter
      //new WaitCommand(1),                                               // wait for ball to shoot
      new UptakeSetPercentOutput(0.3, false, uptake, log),              // make sure uptake is running just in case ball is jammed
      //new WaitCommand(0.5),                                             // wait for ball to shoot
      new FeederSetPercentOutput(0, feeder, log),                       // turn off feeder

      // turn towards ball
      new DriveTurnGyro(TargetType.kAbsolute, 180, 120, 120, 3, driveTrain, limeLight, log).withTimeout(3),

      // deploy intake
      new IntakePistonSetPosition(true, intake, log),
      new IntakeToColorSensor(intake, uptake, log),

      // drive to second ball
      new DriveStraight(AutoConstants.driveToBallTwoInMeters, TargetType.kAbsolute, 180, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),
      
      // turn back to hub
      new DriveTurnGyro(TargetType.kAbsolute, 0, 120, 120, 3, driveTrain, limeLight, log).withTimeout(2),

      // shoot second ball
      new ShooterSetVelocity(InputMode.kSpeedRPM, AutoConstants.ballTwoRPM, shooter, log),  // turn on the shooter
      new FeederSetPercentOutput(0.3, feeder, log),                     // turn on feeder to send first ball to shooter
      new WaitCommand(0.5),                                             // wait for ball to shoot
      new UptakeSetPercentOutput(0.3, false, uptake, log),              // make sure uptake is running just in case ball is jammed
      new WaitCommand(0.5),                                             // wait for ball to shoot
      new FeederSetPercentOutput(0, feeder, log),                       // turn off feeder
      //new DriveTurnGyro(TargetType.kAbsolute, 180, 120, 1200, 3, driveTrain, limeLight, log).withTimeout(3),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.centerBallToBackFourball.value], driveTrain, log),
      // new DriveTurnGyro(TargetType.kAbsolute, 180, 120, 1200, 3, driveTrain, limeLight, log).withTimeout(3),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.backToCenterFourBall.value], driveTrain, log),
      // new ShootSetup(true, 500, pivisionhub, shooter, log),
      // new ShootBall(shooter, uptake, feeder, log),
      // // turn back to loading station      new DriveTurnGyro(TargetType.kAbsolute, 0, 120, 1200, 3, driveTrain, limeLight, log).withTimeout(2),

      new FileLogWrite(false, false, "AutoFourBall", "end", log)

    );
  }
}
