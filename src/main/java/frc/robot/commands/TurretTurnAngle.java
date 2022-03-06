/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.TargetType;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.MathBCR;

import static frc.robot.Constants.TurretConstants.*;

public class TurretTurnAngle extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private Turret turret; // reference to turret subsystem
  private FileLog log;
  private double target; // how many more degrees to the right to turn
  private double maxVel; // max velocity, between 0 and kMaxAngularVelocity in Constants
  private double maxAccel; // max acceleration, between 0 and kMaxAngularAcceleration in Constants
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startAngle, targetRel; // starting angle in degrees, target angle relative to start angle
  private double currAngle, currVelocity;
  private double timeSinceStart;
  private TargetType targetType;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private boolean encoderCalibrated = true;

  // private LimeLight limeLight;     //TODO
  private PiVisionHub piVisionHub;
  private PIDController pidAngVel;
  private double angleTolerance;
  private boolean feedbackUsingVision;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNow; // next state of the system (next loop cycle) as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateForecast; // state of the system in the future, as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * Turns the robot to a target angle.
   * <p> This is the EASIEST constructor to use for the TurrentTurnAngle command.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVision (use limelight to turn towards the goal)   //TODO
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVision]
   * @param angleTolerance the tolerance to use for turn gyro
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    this(type, target, kClampTurnVelocity, kMaxTurnAcceleration, true, angleTolerance, turret, piVisionHub, log);
    // this(type, target, kClampTurnVelocity, kMaxTurnAcceleration, true, angleTolerance, turret, log);
  }

  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVision (use limelight to turn towards the goal)   //TODO
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVision]
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param angleTolerance the tolerance to use for turn gyro
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double maxVel, double maxAccel, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    this(type, target, maxVel, maxAccel, true, angleTolerance, turret, piVisionHub, log);
    // this(type, target, maxVel, maxAccel, true, angleTolerance, turret, log);
  }

  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVision (use pivisionhub to turn towards the goal)     //TODO
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param regenerate true to regenerate profile while running
   * @param angleTolerance the tolerance to use for turn gyro
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double maxVel, double maxAccel, boolean regenerate, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.piVisionHub = piVisionHub;
    this.log = log;
    this.target = MathBCR.normalizeAngle(target);
    this.targetType = type;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, kClampTurnVelocity);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kMaxTurnAcceleration);
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    this.angleTolerance = Math.abs(angleTolerance);

    addRequirements(turret, piVisionHub);
    // addRequirements(turret);

    pidAngVel = new PIDController(kPTurn, 0, kDTurn);
  }

  /**
   * To be used when changing the target value directly from shuffleboard (not a pre-coded target)
   * @param fromShuffleboard true means the value is being changed from shuffleboard
   */
  public TurretTurnAngle(TargetType type, boolean regenerate, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.piVisionHub = piVisionHub;
    this.log = log;
    this.target = 0;
    this.targetType = type;
    this.maxVel = 0;
    this.maxAccel = 0;
    this.regenerate = regenerate;
    this.fromShuffleboard = true;
    this.angleTolerance = 0;
    addRequirements(turret);

    SmartDashboard.putNumber("TurnTurret Manual Target Ang", 45);
    SmartDashboard.putNumber("TurnTurret Manual MaxVel", 150);
    SmartDashboard.putNumber("TurnTurret Manual MaxAccel", kMaxTurnAcceleration);
    SmartDashboard.putNumber("TurnTurret Manual Tolerance", 0.5);

    pidAngVel = new PIDController(kPTurn, 0, kDTurn);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Do not execute if the turret is not calibrated
    encoderCalibrated = turret.isEncoderCalibrated();
    if (!encoderCalibrated) return;
    
    feedbackUsingVision = false;

    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("TurnTurret Manual Target Ang", 45);
      maxVel = SmartDashboard.getNumber("TurnTurret Manual MaxVel", 150);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, kClampTurnVelocity);
      maxAccel = SmartDashboard.getNumber("TurnTurret Manual MaxAccel", kMaxTurnAcceleration);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kMaxTurnAcceleration);
      angleTolerance = SmartDashboard.getNumber("TurnTurret Manual Tolerance", 0.5);
      angleTolerance = Math.abs(angleTolerance);
    }
    // If constants were updated from Shuffleboard, then update PID
    pidAngVel.setPID(kPTurn, 0, kDTurn);
    pidAngVel.reset();


    startAngle = turret.getTurretPosition();

    switch (targetType) {
      case kRelative:
        targetRel = target;
        break;
      case kAbsolute:
        target = MathBCR.normalizeAngle(target);
        if (target>softLimitFwd) target = softLimitFwd;
        if (target<softLimitRev) target = softLimitRev;
        targetRel = target - startAngle;
        break;
      case kVision:
        targetRel = target;
        // targetRel = MathBCR.normalizeAngle(piVisionHub.getXOffset());
        piVisionHub.enableFastLogging(true);
        break;
    }

    // Prevent turret from wrapping
    if ( (startAngle + targetRel)>softLimitFwd )  targetRel = softLimitFwd - startAngle;
    if ( (startAngle + targetRel)<softLimitRev )  targetRel = softLimitRev - startAngle;

    // Set initial and final states
    tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, turret.getTurretVelocity()); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    // Clamp acceleration for short turns
    if ( Math.abs(targetRel) <= kShortTurn) maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kClampAccelShortTurn);

    // initialize velocity and accel limits
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel , maxAccel);
    // generate profile
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;

    log.writeLog(false, "TurretTurnAngle", "initialize", "Total Time", tProfile.totalTime(), "StartAngleAbs", startAngle, "TargetAngleRel", targetRel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do not execute if the turret is not calibrated
    if (!encoderCalibrated) return;
    
    double pFF, pFB, pDB;  // variables for feed forward power, feedback power, and deadband power

    currProfileTime = System.currentTimeMillis();
    // currAngle is relative to the startAngle
    currAngle = turret.getTurretPosition() - startAngle;
    currVelocity = turret.getTurretVelocity();
    
    // if (targetType == TargetType.kVision) {  //TODO
    //   targetRel = MathBCR.normalizeAngle(currAngle + limeLight.getXOffset());
    //   tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
    //   if(limeLight.canTakeSnapshot()) {
    //     limeLight.setSnapshot(true);
    //   }    
    // }

    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNow = tProfile.calculate(timeSinceStart);        // This is where the robot should be now
    tStateForecast = tProfile.calculate(timeSinceStart + tLagTurn);  // This is where the robot should be next cycle (or farther in the future if the robot has lag or backlash)

    // if(tProfile.isFinished(timeSinceStart) && targetType == TargetType.kVision){    //TODO
    //   // If we completed the trapezoid profile and we are using vision, then
    //   // fine-tune angle using feedback based on live camera feedback
    //   feedbackUsingVision = true;
    // }

    targetVel = tStateNow.velocity;
    targetAccel = tStateNow.acceleration;
    double forecastVel = tStateForecast.velocity;
    double forecastAccel = MathUtil.clamp((forecastVel-targetVel)/tLagTurn, -maxAccel, maxAccel);    

    if(!tProfile.isFinished(timeSinceStart)){
      // Feed-forward percent voltage to drive motors
      pFF = (forecastVel * kVTurn) + (forecastAccel * kATurn);
      // Normal feedback for following trapezoid profile
      pFB = MathUtil.clamp(pidAngVel.calculate(currVelocity, targetVel) + kITurn * (tStateNow.position - currAngle), -0.1, 0.1);
    } else {
      // We are past the end of the trapezoid profile
      forecastVel = 0.0;
      forecastAccel = 0.0;
      pFF = 0.0;

      if (targetType == TargetType.kVision) {
        // Live camera feedback
        // TODO make the fixed pFB speed below (0.04) a constant.  Increase value slowly if the robot is not moving.
        // pFB = 0.04 * Math.signum( MathBCR.normalizeAngle(limeLight.getXOffset()) );
        pFB = kITurnEnd * Math.signum( targetRel - currAngle );
      } else {
        // Bang-bang control
        // TODO make the fixed pFB speed below (0.04) a constant.  Increase value slowly if the robot is not moving.
        pFB = kITurnEnd * Math.signum( targetRel - currAngle );
      }
    }
    
    // Use kS to account for dead band
    pDB = kSTurn * Math.signum(pFF + pFB);

    turret.setPercentOutput(+pFF + pFB + pDB);

    if (regenerate) {
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, currVelocity);
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, targetVel);  
      // tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

      // When regenerating using tStateNow, the profile only changes if the final state has moved (such as due to vision)
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateNow);
      profileStartTime = currProfileTime;
    }

    log.writeLog(false, "TurretTurnAngle", "profile", "target", targetRel, 
      "posT", tStateNow.position, "velT", targetVel, "accT", targetAccel,
      "posF", tStateForecast.position, "velF", forecastVel, "accF", forecastAccel,
      "posA", currAngle, "velA", currVelocity, "pFF", pFF, "pFB", pFB, "pTotal", pFF+pFB+pDB
      // , "LL x", limeLight.getXOffset(), "LL y", limeLight.getYOffset()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
    
    // if (targetType == TargetType.kVision) {      //TODO
    //   limeLight.enableFastLogging(false);
    // }
    
    log.writeLog(false, "TurretTurnAngle", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do not execute if the turret is not calibrated
    if (!encoderCalibrated) return true;

    // if ((targetType != TargetType.kVision && Math.abs(targetRel - currAngle) < angleTolerance) || 
    //     (targetType == TargetType.kVision && limeLight.seesTarget() && Math.abs(limeLight.getXOffset()) < angleTolerance) ||
    //     (targetType == TargetType.kVision && !limeLight.seesTarget() && feedbackUsingVision) ) {
    if (Math.abs(targetRel - currAngle) < angleTolerance) {
      accuracyCounter++;
      log.writeLog(false, "TurretTurnAngle", "WithinTolerance", "Target Ang", targetRel, "Actual Ang", currAngle, 
        // "LimeLight Xoff", limeLight.getXOffset(), 
        "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}