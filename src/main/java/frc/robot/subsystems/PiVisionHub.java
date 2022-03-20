/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.FileLog;

public class PiVisionHub extends PiVision {
  private boolean setFlashAuto = true;
  private PowerDistribution pd;
  
  public PiVisionHub(PowerDistribution pd, FileLog log) {
    super("shooter-cam", log);
    this.pd = pd;
  }

  public void setLEDState(boolean state) {
    pd.setSwitchableChannel(state);
  }

  public boolean LEDState() {
    return pd.getSwitchableChannel();
  }

  /**
   * Takes into account not being in line with the target.
   * @return distance from camera to target, on the floor, in feet
   */
  public double getDistance() {    //  TODO  this could return a erroneous value if vision misses a frame or is temporarily blocked.  Use avgrging or filtering
    // if (50 < width && width < 300) return -1.3974*width+359.833;
    // if (50 < width && width < 300) return -1.2336*width+327.002;
    if (width == 0) return 0; // TODO change if certain width range or change if certain number of targets
    if (numberOfTargets < 4) return numberOfTargets;
    // return 27983.4 / width - 51.4736;  
    return 26691.7 / width - 60.0122;  
  }

  @Override
  public void periodic() {
    super.periodic();
    
    if (log.getLogRotation() == log.PIVISION_CYCLE) {
      SmartDashboard.putNumber("cam-shooter Distance", getDistance());
    }
  }
  /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  @Override
  public void updatePiVisionLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, name, "Update Variables", 
      "Number of Targets", numberOfTargets,
      "Width", width,
      "X Offset", x,
      // "Target Area", area,
      // "Latency", latency,
      "Network Table Read Counter", networkTableReadCounter,
      // "Snapshot Count", snapshotCount,
      "Dist", getDistance()
      );
  }
}
