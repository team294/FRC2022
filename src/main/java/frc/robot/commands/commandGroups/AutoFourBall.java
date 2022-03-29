package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;


public class AutoFourBall extends SequentialCommandGroup {

/**
 * Four ball auto starting in the center facing the ball with hub directly behind lined up for a shot
 * 
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoFourBall(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, Turret turret, TrajectoryCache trajectoryCache, PiVisionHub pivisionhub, LimeLight limeLight, FileLog log) {
    addCommands(
      // setup
      new FileLogWrite(false, false, "AutoFourBall", "starting", log),
      new DriveResetPose(0, 0, 180, driveTrain, log),   // set initial pose to 180 degrees away from the hub (facing the ball)
      new PiVisionHubSetLEDState(1, pivisionhub),       // turn on led light for vision

      // intake one ball, turn, shoot then go to the back and get two more, come back and shoot
      
      // get ready to intake ball
      new IntakePistonSetPosition(true, intake, log),
      new IntakeToColorSensor(intake, uptake, log),

      // drive to second ball
      new DriveStraight(AutoConstants.driveToBallTwoInMeters, TargetType.kAbsolute, 180, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // turn back to hub
      new DriveTurnGyro(TargetType.kAbsolute, 0, 300, 200, 3, driveTrain, limeLight, log).withTimeout(2),

      // shoot 
      new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log),
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(intake, uptake, feeder, shooter, log),

      // turn on uptake for next set of balls
      new UptakeSetPercentOutput(0.25, false, uptake, log),

      // drive to back balls
      new DriveTurnGyro(TargetType.kAbsolute, 168, 300, 200, 3, driveTrain, limeLight, log).withTimeout(3),
      // new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.centerBallToBackFourball.value], driveTrain, log),
      new DriveStraight(4.2, TargetType.kAbsolute, 168, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),
      
      // wait for human player to feed ball
      new WaitCommand(0.5), 
      new IntakePistonSetPosition(false, intake, log),

      // drive back to sweet spot
      new DriveTurnGyro(TargetType.kAbsolute, 0, 300, 200, 3, driveTrain, limeLight, log).withTimeout(3),
      // new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.backToCenterFourBall.value], driveTrain, log),
      new DriveStraight(3.5, TargetType.kAbsolute, 0, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // shoot
      new TurretTurnAngle(TargetType.kVisionScanRight, 0, 3, turret, pivisionhub, log).withTimeout(1),
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(intake, uptake, feeder, shooter, log),

      new FileLogWrite(false, false, "AutoFourBall", "end", log)

    );
  }
}
