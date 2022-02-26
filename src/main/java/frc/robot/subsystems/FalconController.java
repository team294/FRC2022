/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;
import static frc.robot.Constants.*;


public class FalconController extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX motor;
  
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private double encoderZero = 0.0;     // Zero position for encoder
  private int timeoutMs = 0; // was 30, changed to 0 for testing

  // Velocity control variables
  private double kS, kV;      // Feed forward parameters
  private boolean velocityControlOn = false;    // Is this subsystem using velocity control currently?
  private double measuredRPM = 0.0;             // Current measured speed
  private double setpointRPM = 0.0;             // Current velocity setpoint

  
  public FalconController(int CANPort, String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    motor = new WPI_TalonFX(CANPort);

    // set motor configuration
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    motor.configNeutralDeadband(0.01);
    motor.configVoltageCompSaturation(FalconConstants.compensationVoltage);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.05);   //seconds from neutral to full
    motor.configClosedloopRamp(0.05); //seconds from neutral to full

    // set sensor configuration
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    motor.setSensorPhase(false);

    zeroEncoder();

    // PID coefficients initial
    setPIDSV(FalconConstants.kP, FalconConstants.kI, FalconConstants.kI, 
      FalconConstants.kS, FalconConstants.kV);

    stopMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets the voltage of the motor
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
    motor.setVoltage(voltage);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param percent percent
   */
  public void setMotorPercentOutput(double percent){
    motor.set(ControlMode.PercentOutput, percent);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
  * Stops the motor
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }


  /**
   * Returns the motor position
   * @return position of motor in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    return motor.getSelectedSensorPosition(0);
  }

  /**
   * Returns the motor position
   * @return position of motor in revolutions
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
   * @return velocity of motor in rpm
   */
  public double getMotorVelocity(){
    return motor.getSelectedSensorVelocity(0)*FalconConstants.rawVelocityToRPM;
  }


  /**
   * Sets PIDSV parameters for motor velocity control.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / FalconConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * @param P Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * @param I Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * @param D Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * @param S Feedforward offset, in percent output (-1 to +1)
   * @param V Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   */
  public void setPIDSV(double P, double I, double D, double S, double V)  {
    // set PID coefficients
    motor.config_kP(0, P, timeoutMs);
    motor.config_kI(0, I, timeoutMs);
    motor.config_kD(0, D, timeoutMs);
    // motor.config_kF(0, F, timeoutMs);  // value = 1023 * desired-percent-out / at-sensor-velocity-sensor-units-per-100ms
    kS = S;
    kV = V;

    if (velocityControlOn) {
      // Reset velocity to force kS and kV updates to take effect
      setMotorVelocity(setpointRPM);
    }
  }

  /**
   * Run motor in a velocity closed loop mode.
   * @param motorRPM setPoint RPM
   */
  public void setMotorVelocity(double motorRPM) {
    velocityControlOn = true;
    setpointRPM = motorRPM;
    
    motor.set(ControlMode.Velocity, setpointRPM/FalconConstants.rawVelocityToRPM,
      DemandType.ArbitraryFeedForward, kS*Math.signum(setpointRPM) + kV*setpointRPM);
  }
  
  /**
   * @return PID error, in RPM.  + = actual speed too fast, - = actual speed too slow
   */
  public double getVelocityPIDError() {
    return measuredRPM - setpointRPM;
  }


  @Override
  public void periodic(){
    measuredRPM = getMotorVelocity();
    if(fastLogging || log.getLogRotation() == log.FALCON_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), motor.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getMotorPosition());
      SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), measuredRPM);
      SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), motor.getTemperature());
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
      "Bus Volt", motor.getBusVoltage(),
      "Out Percent", motor.getMotorOutputPercent(),
      "Volt", motor.getMotorOutputVoltage(), 
      "Amps", motor.getSupplyCurrent(),
      "Temperature", motor.getTemperature(),
      "Position", getMotorPosition(),
      "Measured RPM", measuredRPM,
      "Setpoint RPM", setpointRPM
    );
  }

  
}
