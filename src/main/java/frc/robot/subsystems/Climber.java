package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.Constants.*;


public class Climber extends SubsystemBase implements Loggable {
  protected final FileLog log;
  private final Solenoid leftPiston;
  private final Solenoid rightPiston;

  private MechanismLigament2d lPiston;
  private MechanismLigament2d lArm;
  private MechanismLigament2d rPiston;
  private MechanismLigament2d rArm;
  
  private long time = 0, startTime;

  private boolean extending = false, extended = false, retracted = true, retracting = false;
  protected boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard
  
  /**
   * Creates a generic intake (front or rear)
   * @param subsystemName  String name for subsystem
   * @param log
   */
  public Climber(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    leftPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbLeft);
    rightPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbRight);
  }

  /**
   * Creates a generic intake (front or rear)
   * @param subsystemName  String name for subsystem
   * @param log
   */
  public Climber(String subsystemName, FileLog log, boolean useReal) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    leftPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbLeft);
    rightPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbRight);
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  public FileLog getFileLog() {
    return log;
  }

  /**
   * Sets if the piston should be extended or not
   * 
   * @param value true = extend, false = retract
   */
  public void setPistonExtended(boolean value) {
    leftPiston.set(value);
    rightPiston.set(value);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = extended, false = retracted
   */
  public boolean getPistonExtended() {
    return leftPiston.get() && rightPiston.get();
  }

  public boolean getLeftPistonExtended() {
    return leftPiston.get();
  }

  public boolean getRightPistonExtended() {
    return rightPiston.get();
  }

  /**
   * Toggles the piston between retracted and deployed
   */
  public void togglePistonExtended(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", getPistonExtended());
    setPistonExtended(!getPistonExtended());
  }

  public void initSim() {
    Mechanism2d lMech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d lRoot = lMech.getRoot("climber", 2, 0);

    lPiston = lRoot.append(new MechanismLigament2d("lPiston", ClimberConstants.climberMinimumLength, 90));
    lArm =
        lPiston.append(
            new MechanismLigament2d("lArm", 0.5, 180, 6, new Color8Bit(Color.kPurple)));
            
    // post the mechanism to the dash board
    SmartDashboard.putData("Mech2d", lMech);
  }

  @Override
  public void simulationPeriodic(){
    long currTime = System.currentTimeMillis();
    if (getPistonExtended() && !extended && !extending) {
      startTime = currTime;
      extending = true;
    }
    if (extending) {
      double timeDiff = (currTime-startTime);
      if (timeDiff > ClimberConstants.msToExtend) {
        extended = true;
        extending = false;
        retracted = false;
        return;
      }
      lPiston.setLength(ClimberConstants.climberMinimumLength + Math.min(timeDiff/ClimberConstants.msToExtend, 1) * ClimberConstants.climberExtendLength);
      lArm.setAngle(180 - timeDiff/ClimberConstants.msToExtend*180);
    }
    if (!getPistonExtended() && !retracted && !retracting) {
      startTime = currTime;
      retracting = true;
    }
    if (retracting) {
      double timeDiff = (currTime-startTime);
      if (timeDiff > ClimberConstants.msToExtend) {
        retracted = true;
        extended = false;
        retracting = false;
        return;
      }
      lPiston.setLength(ClimberConstants.climberMinimumLength + ClimberConstants.climberExtendLength - Math.min(timeDiff/ClimberConstants.msToExtend, 1) * ClimberConstants.climberExtendLength);
      lArm.setAngle(timeDiff/ClimberConstants.msToExtend*180);
    }

    // lPiston.setLength(ClimberConstants.climberMinimumLength + ((getLeftPistonExtended()) ? ClimberConstants.climberExtendLength : 0));
    // lArm.setAngle((getLeftPistonExtended()) ? 0 : 180);
  }


  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

}
