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
import frc.robot.commands.PiVisionHubSetLEDState;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.TurretTurnAngle;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.commands.DriveFollowTrajectory.PIDType;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


public class AutoFourBall extends SequentialCommandGroup {

/**
 * Four ball auto starting in the center facing the ball with hub directly behind lined up for a shot
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
public AutoFourBall(double waitTime, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, Turret turret, TrajectoryCache trajectoryCache, PiVisionHub pivisionhub, LimeLight limeLight, FileLog log) {
    addCommands(
      // setup
      new FileLogWrite(false, false, "AutoFourBall", "starting", log),
      new WaitCommand(waitTime),                        // delay from shuffleboard if needed
      new DriveResetPose(0, 0, 180, driveTrain, log),   // set initial pose to 180 degrees away from the hub (facing the ball)
      new PiVisionHubSetLEDState(1, pivisionhub),       // turn on led light for vision

      // intake one ball, turn, shoot then go to the back and get two more, come back and shoot
      
      // intake ball
      new IntakePistonSetPosition(true, intake, log),
      new IntakeToColorSensor(intake, uptake, log),

      // drive to second ball
      new DriveStraight(AutoConstants.driveToBallTwoInMeters, TargetType.kAbsolute, 180, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // turn back to hub
      new DriveTurnGyro(TargetType.kAbsolute, 0, 120, 120, 3, driveTrain, limeLight, log).withTimeout(2),

      // shoot 
      // new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log),
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(intake, uptake, feeder, log),

      // drive to back balls
      // new DriveTurnGyro(TargetType.kAbsolute, 180, 120, 1200, 3, driveTrain, limeLight, log).withTimeout(3),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.centerBallToBackFourball.value], driveTrain, log),

      // drive to center shooting position
      // new DriveTurnGyro(TargetType.kAbsolute, 180, 120, 1200, 3, driveTrain, limeLight, log).withTimeout(3),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.backToCenterFourBall.value], driveTrain, log),
      new IntakePistonSetPosition(false, intake, log),

      // shoot
      // new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log),
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(intake, uptake, feeder, log),

      new FileLogWrite(false, false, "AutoFourBall", "end", log)

    );
  }
}
