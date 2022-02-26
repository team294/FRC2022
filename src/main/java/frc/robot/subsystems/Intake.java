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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;
import static frc.robot.Constants.*;


public class Intake extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX Intake;
  
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private double encoderZero = 0.0;     // Zero position for encoder
  private int timeoutMs = 0; // was 30, changed to 0 for testing

  private double measuredRPM = 0.0;             // Current measured speed
  private double setpointRPM = 0.0;             // Current velocity setpoint

  
  public Intake(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    Intake = new WPI_TalonFX(Ports.CANIntake);

    // set Intake configuration
    Intake.configFactoryDefault();
    Intake.setInverted(false);
    Intake.setNeutralMode(NeutralMode.Brake);
    Intake.configPeakOutputForward(1.0);
    Intake.configPeakOutputReverse(-1.0);
    Intake.configNeutralDeadband(0.01);
    Intake.configVoltageCompSaturation(FalconConstants.compensationVoltage);
    Intake.enableVoltageCompensation(true);
    Intake.configOpenloopRamp(0.05);   //seconds from neutral to full
    Intake.configClosedloopRamp(0.05); //seconds from neutral to full

    // set sensor configuration
    Intake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    Intake.setSensorPhase(false);

    zeroEncoder();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets the voltage of the Intake
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage
   */
  public void setVoltage(double voltage) {
    Intake.setVoltage(voltage);
    setpointRPM = 0.0;
  }

  /**
   * sets the percent of the Intake, using voltage compensation if turned on
   * @param percent percent
   */
  public void setMotorPercentOutput(double percent){
    Intake.set(ControlMode.PercentOutput, percent);
    setpointRPM = 0.0;
  }

  /**
  * Stops the Intake
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
    setpointRPM = 0.0;
  }


  /**
   * Returns the Intake position
   * @return position of Intake in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    return Intake.getSelectedSensorPosition(0);
  }

  /**
   * Returns the Intake position
   * @return position of Intake in revolutions
   */
  public double getMotorPosition(){
    return (getMotorPositionRaw() - encoderZero)/FalconConstants.ticksPerRevolution;
  }

  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  /**
   * @return velocity of Intake in rpm
   */
  public double getMotorVelocity(){
    return Intake.getSelectedSensorVelocity(0)*FalconConstants.rawVelocityToRPM;
  }

  /**
   * Run Intake in a velocity closed loop mode.
   * @param motorRPM setPoint RPM
   */
  public void setMotorVelocity(double velocity) {
    Intake.set(ControlMode.Velocity, velocity);
  }


  @Override
  public void periodic(){
    measuredRPM = getMotorVelocity();
    if(fastLogging || log.getLogRotation() == log.FALCON_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), Intake.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getMotorPosition());
      SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), measuredRPM);
      SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), Intake.getTemperature());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", Intake.getBusVoltage(),
      "Out Percent", Intake.getMotorOutputPercent(),
      "Volt", Intake.getMotorOutputVoltage(), 
      "Amps", Intake.getSupplyCurrent(),
      "Temperature", Intake.getTemperature(),
      "Position", getMotorPosition(),
      "Measured RPM", measuredRPM,
      "Setpoint RPM", setpointRPM
    );
  }

  
}
