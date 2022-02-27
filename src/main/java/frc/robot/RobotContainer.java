
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.TargetType;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.commands.commandGroups.*;
import frc.robot.subsystems.*;
import frc.robot.triggers.*;
import frc.robot.utilities.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FileLog log = new FileLog("A1");
  private final TemperatureCheck tempCheck = new TemperatureCheck(log);
  private final PowerDistribution powerdistribution = new PowerDistribution(0, ModuleType.kRev);
  private final DriveTrain driveTrain = new DriveTrain(log, tempCheck);
  private final Shooter shooter = new Shooter(log);
  private final Feeder feeder = new Feeder("Feeder", log);
  private final Uptake uptake = new Uptake("Uptake",log);
  private final Intake intake = new Intake("Intake",log);
  private final Turret turret = new Turret(log);
  private final PiVisionHub pivisionhub = new PiVisionHub(powerdistribution, log); //Pi ip: 10.2.94.21

  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, log);

  private final Joystick xboxController = new Joystick(OIConstants.usbXboxController);//assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private boolean rumbling = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick, log));
  }

  /**
   * Define Shuffleboard mappings.
   */
  public void configureShuffleboard() {

    // display sticky faults
    RobotPreferences.showStickyFaults();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // display overheating motors
    tempCheck.displayOverheatingMotors();

    // Shooter widgets
    SmartDashboard.putData("Shooter Stop", new ShooterStop(shooter, log));
    SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercentOutput(shooter, log));
    SmartDashboard.putData("Shooter Set PID", new ShooterSetPIDSV(shooter, log));
    SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(InputMode.kSpeedRPM, shooter, log));
    SmartDashboard.putData("Shooter RPM from Distance", new ShooterSetVelocity(InputMode.kDistFeet, shooter, log));
    SmartDashboard.putData("Shooter Calibrate Fwd", new ShooterRampOutput(0, 0.9, 30.0, shooter, log));
    SmartDashboard.putData("Shooter Distance to RPM", new ShooterDistToRPM(shooter, log));

    SmartDashboard.putData("Set Feeder Percent", new FeederSetPercentOutput(feeder, log));
    SmartDashboard.putData("Stop Feeder", new FeederStop(feeder, log));

    SmartDashboard.putData("Start Feeder Sequence", new FeederBallReadyToShoot(feeder, log));
    SmartDashboard.putData("Shoot Red Ball Sequence", new FeedAndShootBall(shooter, feeder, uptake, log, BallColor.kBlue));
    SmartDashboard.putData("Shoot Blue Ball Sequence", new FeedAndShootBall(shooter, feeder, uptake, log, BallColor.kRed));
    
    SmartDashboard.putData("Uptake Run Upward", new UptakeSetPercentOutput(.25, false, uptake, log));
    SmartDashboard.putData("Uptake Eject Ball", new UptakeSetPercentOutput(.25, true, uptake, log));
    SmartDashboard.putData("Uptake Stop", new UptakeStop(uptake, log));
    SmartDashboard.putData("Uptake Reject Blue", new UptakeSortBall(BallColor.kBlue, uptake, feeder, log));
    SmartDashboard.putData("Uptake Reject Red", new UptakeSortBall(BallColor.kRed, uptake, feeder, log));

    SmartDashboard.putData("Turret Stop", new TurretStop(turret, log));
    SmartDashboard.putData("Turret Set Percent", new TurretSetPercentOutput(turret, log));
    SmartDashboard.putData("Turret Calibrate Fwd", new TurretRampOutput(0, 0.3, 10.0, turret, log));  
    SmartDashboard.putData("Turret Turn to Angle", new TurretTurnAngle(TargetType.kAbsolute, false, turret, log));

    SmartDashboard.putData("shooter-cam ToggleLED", new PiVisionHubToggleLED(pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOn", new PiVisionHubLEDOn(pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOff", new PiVisionHubLEDOff(pivisionhub));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    JoystickButton[] xb = new JoystickButton[11];
    //check povtrigger and axis trigger number bindings
    Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    
    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }
    

    xb[1].whenPressed(new ShootBall(0,shooter, uptake,log));//a -short
    xb[2].whenPressed(new ShootBall(0,shooter,uptake,log));//b -medium
    // xb[3].whenHeld();//x -auto?
    xb[4].whenPressed(new ShootBall(0,shooter,uptake,log));//y -long
    //xb[11].whenHeld(); //l1, turn turret left
    //xb[12].whenHeld(); //r1, turn turret right
    //xb[7].whenHeld(); //start, toggle rollers
    //xb[8].whenHeld(); //start, toggle lights
    //xbPOVLeft.whenHeld();//climb

  }

  /**
   * Define Joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // joystick left button
    // left[1].whenPressed(new Wait(0));
    // right[1].whenPressed(new Wait(0));

    // joystick right button
    // left[2].whenHeld(new VisionAssistSequence(driveTrain, limeLightGoal, log, shooter, feeder, led, hopper, intake));
    // right[2].whileHeld(new DriveTurnGyro(TargetType.kVision, 0, 90, 100, true, 1, driveTrain, limeLightGoal, log)); // turn gyro with vision
    // right[1].whenHeld(new DriveJogTurn(true,  driveTrain, log ));
    // left[1].whenHeld(new DriveJogTurn(false,  driveTrain, log ));
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    // coP[1].whenPressed(new ClimbPistonsSetPosition(true, climb, log)); // deploy climb pistons
    // coP[2].whenPressed(new ClimbPistonsSetPosition(false, climb, log)); // retract climb pistons

    // coP[3].whenPressed(new ClimbSetVelocity(false, ClimbConstants.latchHeight, climb, log)); // raise climb arms to default latching height
    // coP[4].whenPressed(new ClimbSetVelocity(false, ClimbConstants.latchExtensionHeight, climb, log)); // raise climb arms to slightly above default latching height

    // coP[5].whenHeld(new ClimbSetPercentOutput(0.4, climb, log)); // manually raise climb arms, slowly
    // coP[6].whenHeld(new ClimbSetPercentOutput(-0.4, climb, log)); // manually lower climb arms, slowly
    
    // top row RED SWITCH
    // coP[8].whenPressed(new ClimbLiftSequence(climb, led, log)); // climb lift sequence (rainbow LEDs and climb arms lower to lifting height)

    // middle row UP then DOWN, from LEFT to RIGHT
    // coP[9].whenHeld(new ClimbLeftSetPercentOutput(0.4, climb, log)); // manually raise left climb arm, slowly
    // coP[10].whenHeld(new ClimbLeftSetPercentOutput(-0.4, climb, log)); // manually lower left climb arm, slowly

    // coP[11].whenHeld(new ClimbRightSetPercentOutput(0.4, climb, log)); // manually raise right climb arm, slowly
    // coP[12].whenHeld(new ClimbRightSetPercentOutput(-0.4, climb, log)); // manually lower right climb arm, slowly

    // coP[13].whenHeld(new ClimbSetPercentOutput(0.8, climb, log)); // manually raise climb arms, quickly
    // coP[14].whenHeld(new ClimbSetPercentOutput(-0.8, climb, log)); // manually lower climb arms, quickly

    // middle row UP OR DOWN, fourth button
    // coP[7].whenPressed(new ShooterSetVoltage(0, shooter, log)); // stop shooter

    // bottom row UP then DOWN, from LEFT to RIGHT
    // coP[15].whenPressed(new ClimbPistonUnlock(false, climb)); // lock climb lock
    // coP[16].whenPressed(new ClimbPistonUnlock(true, climb)); // unlock climb lock
  }


  /**
   * Sets the rumble on the XBox controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);

    if (percentRumble == 0) rumbling = false;
    else rumbling = true;
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(driveTrain, shooter, feeder, intake, 
      // limeLightGoal, limeLightBall, led,
      log );
  }

  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    boolean error = true;
    if ((driveTrain.getLeftBusVoltage() > 8.0) && (driveTrain.isGyroReading() == true) && (driveTrain.getRightTemp() > 5.0)) error = false;    // The CAN bus and Gyro are working
  
    if (error) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }  //    TODO May want to flash this
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");
   
    driveTrain.setDriveModeCoast(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false);
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
    // if (limeLightGoal.seesTarget()) {
    //   setXBoxRumble(1.0);
    // } else if (intake.intakeGetPercentOutput() == Math.abs(Constants.IntakeConstants.intakeDefaultPercentOutput)) {
    //   setXBoxRumble(0.4);
    // } else {
    //   setXBoxRumble(0);
    // }
  }
}
