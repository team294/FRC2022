
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
import frc.robot.subsystems.*;
import frc.robot.triggers.AxisTrigger;
import frc.robot.triggers.POVTrigger;
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
  private final PowerDistribution m_powerdistribution = new PowerDistribution(0, ModuleType.kRev);
  private final Shooter m_shooter = new Shooter(log);
  private final Feeder m_feeder = new Feeder("Feeder", log);
  private final Uptake m_uptake = new Uptake("Uptake",log);
  private final Turret m_turret = new Turret(log);
  private final PiVisionHub m_pivisionhub = new PiVisionHub(m_powerdistribution, log); //Pi ip: 10.2.94.21
  
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick xboxController = new Joystick(OIConstants.usbXboxController);//assuming usbxboxcontroller is int

  private int displayCount = 1;
  private boolean rumbling = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure Shuffleboard
    SmartDashboard.putData("Shooter Stop", new ShooterStop(m_shooter, log));
    SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercentOutput(m_shooter, log));
    SmartDashboard.putData("Shooter Set PID", new ShooterSetPIDSV(m_shooter, log));
    SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(InputMode.kSpeedRPM, m_shooter, log));
    SmartDashboard.putData("Shooter RPM from Distance", new ShooterSetVelocity(InputMode.kDistFeet, m_shooter, log));
    SmartDashboard.putData("Shooter Calibrate Fwd", new ShooterRampOutput(0, 0.9, 30.0, m_shooter, log));

    SmartDashboard.putData("Shooter Distance to RPM", new ShooterDistToRPM(m_shooter, log));

    SmartDashboard.putData("Set Feeder Percent", new FeederSetPercentOutput(m_feeder, log));
    SmartDashboard.putData("Stop Feeder", new FeederStop(m_feeder, log));

    SmartDashboard.putData("Start Feeder Sequence", new FeederBallReadyToShoot(m_feeder, log));
    SmartDashboard.putData("Shoot Red Ball Sequence", new FeedAndShootBall(m_shooter, m_feeder, m_uptake, log, BallColor.kBlue));
    SmartDashboard.putData("Shoot Blue Ball Sequence", new FeedAndShootBall(m_shooter, m_feeder, m_uptake, log, BallColor.kRed));
    
    SmartDashboard.putData("Uptake Run Upward", new UptakeSetPercentOutput(.25, false, m_uptake, log));
    SmartDashboard.putData("Uptake Eject Ball", new UptakeSetPercentOutput(.25, true, m_uptake, log));
    SmartDashboard.putData("Uptake Stop", new UptakeStop(m_uptake, log));
    SmartDashboard.putData("Uptake Reject Blue", new UptakeSortBall(BallColor.kBlue, m_uptake, m_feeder, log));
    SmartDashboard.putData("Uptake Reject Red", new UptakeSortBall(BallColor.kRed, m_uptake, m_feeder, log));

    SmartDashboard.putData("Turret Stop", new TurretStop(m_turret, log));
    SmartDashboard.putData("Turret Set Percent", new TurretSetPercentOutput(m_turret, log));
    SmartDashboard.putData("Turret Calibrate Fwd", new TurretRampOutput(0, 0.3, 10.0, m_turret, log));  
    SmartDashboard.putData("Turret Turn to Angle", new TurretTurnAngle(TargetType.kAbsolute, false, m_turret, log));

    SmartDashboard.putData("shooter-cam ToggleLED", new PiVisionHubToggleLED(m_pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOn", new PiVisionHubLEDOn(m_pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOff", new PiVisionHubLEDOff(m_pivisionhub));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons();
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
    

    xb[1].whenPressed(new ShootBall(0,m_shooter, m_uptake,log));//a -short
    xb[2].whenPressed(new ShootBall(0,m_shooter,m_uptake,log));//b -medium
    // xb[3].whenHeld();//x -auto?
    xb[4].whenPressed(new ShootBall(0,m_shooter,m_uptake,log));//y -long
    //xb[11].whenHeld(); //l1, turn turret left
    //xb[12].whenHeld(); //r1, turn turret right
    //xb[7].whenHeld(); //start, toggle rollers
    //xb[8].whenHeld(); //start, toggle lights
    //xbPOVLeft.whenHeld();//climb

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
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    
  }
}
