/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ColorSensor;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.Constants.*;


public class Uptake extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX uptake; // Motor running most wheels in the uptake
  private final WPI_TalonFX eject;  // Motore that selects between uptaking and ejecting
  public final ColorSensor colorSensor;   // Color sensor in uptake
  private DigitalInput ejectSensor = new DigitalInput(Ports.DIOEjectBallSensor) ; // Senses when a ball is ejected

  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private int timeoutMs = 0; // was 30, changed to 0 for testing 
  
  public Uptake(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    uptake = new WPI_TalonFX(Ports.CANUptake);
    eject = new WPI_TalonFX(Ports.CANEject);
    colorSensor = new ColorSensor(log);

    // set uptake configuration
    uptake.configFactoryDefault();
    uptake.setInverted(true);
    uptake.setNeutralMode(NeutralMode.Brake);
    uptake.configPeakOutputForward(1.0);
    uptake.configPeakOutputReverse(-1.0);
    uptake.configNeutralDeadband(0.01);
    uptake.configVoltageCompSaturation(UptakeConstants.compensationVoltage);
    uptake.enableVoltageCompensation(true);
    uptake.configOpenloopRamp(0.05);   //seconds from neutral to full
    uptake.configClosedloopRamp(0.05); //seconds from neutral to full

    // set sensor configuration
    uptake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    uptake.setSensorPhase(false);


    eject.configFactoryDefault();
    eject.setInverted(false);
    eject.setNeutralMode(NeutralMode.Brake);
    eject.configPeakOutputForward(1.0);
    eject.configPeakOutputReverse(-1.0);
    eject.configNeutralDeadband(0.01);
    eject.configVoltageCompSaturation(UptakeConstants.compensationVoltage);
    eject.enableVoltageCompensation(true);
    eject.configOpenloopRamp(0.05);   //seconds from neutral to full
    eject.configClosedloopRamp(0.05); //seconds from neutral to full

    // set sensor configuration
    eject.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    eject.setSensorPhase(false);

    stopMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * 
   * @return true = ball is in ejector
   */
  public boolean isBallInEjector(){
    return !ejectSensor.get();
  }

  /**
   * Sets the voltage of the uptake
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage, + = up, - = down
   * @param ejectBall true = ball path to eject, false = ball path to feeder
   */
  public void setVoltage(double voltage, boolean ejectBall) {
    uptake.setVoltage(voltage);
    eject.setVoltage( ejectBall ? voltage : -voltage );
  }

  /**
   * sets the percent of the uptake, using voltage compensation if turned on
   * @param percent -1.0 to 1.0, + = up, - = down
   * @param ejectBall true = ball path to eject, false = ball path to feeder
   */
  public void setUptakeEjectPercentOutput(double percent,boolean ejectBall){
    uptake.set(ControlMode.PercentOutput, percent);
    eject.set(ControlMode.PercentOutput, ejectBall ? percent : -percent);
  }


    /**
   * sets the percent of the uptake, using voltage compensation if turned on
   * @param percent percent
   */
  public void setUptakePercentOutput(double percent){
    uptake.set(ControlMode.PercentOutput, percent);
    
  }

  

  /**
  * Stops the uptake
  */
  public void stopMotor(){
    setUptakeEjectPercentOutput(0,false);
  }


  /**
   * Returns the eject position
   * @return position of eject in raw units, without software zeroing
   */
  public double getEjectPositionRaw(){
    return eject.getSelectedSensorPosition(0);
  }

   /**
   * Returns the Uptake position
   * @return position of uptake in raw units, without software zeroing
   */
  public double getUptakePositionRaw(){
    return uptake.getSelectedSensorPosition(0);
  }

  /**
   * @return velocity of eject in rpm
   */
  public double getEjectVelocity(){
    return eject.getSelectedSensorVelocity(0)*UptakeConstants.rawVelocityToRPM;
  }

  /**
   * @return velocity of uptake in rpm
   */
  public double getUptakeVelocity(){
    return uptake.getSelectedSensorVelocity(0)*UptakeConstants.rawVelocityToRPM;
  }


  @Override
  public void periodic(){
    if(fastLogging || log.getLogRotation() == log.UPTAKE_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber("Eject Voltage", eject.getMotorOutputVoltage());
      SmartDashboard.putNumber("Uptake Voltage", uptake.getMotorOutputVoltage());
      SmartDashboard.putNumber("Uptake Position Rev", getUptakePositionRaw());
      SmartDashboard.putNumber("Eject Position Rev", getEjectPositionRaw());
      SmartDashboard.putNumber("Eject Velocity RPM", getEjectVelocity());
      SmartDashboard.putNumber("Uptake Velocity RPM", getUptakeVelocity());
      SmartDashboard.putNumber("Eject Temperature C", eject.getTemperature());
      SmartDashboard.putNumber("Uptake Temperature C", uptake.getTemperature());
      SmartDashboard.putBoolean("Uptake Ball Present", colorSensor.isBallPresent());
      SmartDashboard.putBoolean("Eject Ball Present", isBallInEjector());

      colorSensor.updateShuffleboard();
      colorSensor.updateLog(false);
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about uptake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", uptake.getBusVoltage(),
      "Uptake Percent", uptake.getMotorOutputPercent(),
      "Eject Percent", eject.getMotorOutputPercent(),
      "Uptake Volt", uptake.getMotorOutputVoltage(), 
      "Eject Volt", eject.getMotorOutputVoltage(), 
      "Uptake Amps", uptake.getSupplyCurrent(),
      "Eject Amps", eject.getSupplyCurrent(),
      "Uptake Temperature", uptake.getTemperature(),
      "Eject Temperature", eject.getTemperature(),
      "Uptake Position", getUptakePositionRaw(),
      "Eject Position", getEjectPositionRaw(),
      "Uptake RPM", getUptakeVelocity(),
      "Eject RPM", getEjectVelocity(),
      "Uptake Ball Present", colorSensor.isBallPresent(),
      "Eject Ball Present", isBallInEjector(),
      "Uptake Ball color", colorSensor.getBallColorString()
    );
  }

  
}
