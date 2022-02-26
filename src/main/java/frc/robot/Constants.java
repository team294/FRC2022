/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Recognized ball colors from color sensor.
     */
    public enum BallColor {
        kNone(0),
        kBlue(1),
        kRed(2),
        kYellow(3),
        kOther(4);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        BallColor(int value) { this.value = value; }
    }


    /**
     * Options to select driving or turret target types.
     */
    public enum TargetType {
        kRelative(0),
        kAbsolute(1),
        kVision(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TargetType(int value) { this.value = value; }
    }

    public enum BallLocation {
        kIntake(0),
        kUptake(1),
        kFeeder(2),
        kShooter(3);

        public final int value;
        BallLocation(int value){
            this.value = value;
        }

    }

    public static final class RobotConstants {
        // Global constants go here
        
    }

    public static final class Ports {
        // CANbus addresses
        public static final int CANShooter = 21;
        public static final int CANEject = 23;
        public static final int CANUptake = 24;
        public static final int CANFeeder = 11;
        public static final int CANTurret = 25;
        public static final int CANFalcon1 = 31;
        public static final int CANFalcon2 = 32;
        public static final int CANIntake = 0;


        // Digital IO ports
        public static final int DIOFeederBallSensor = 1;
        public static final int DIOEjectBallSensor = 2;
        public static final int DIOTurretCalSwitch = 3;


        // I2C ports
        public static final int I2CcolorSensor = 0x52;       // According to REV docs, color sensor is at 0x52 = 82.  Rob had 39?
    }
   
    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        // public static final int usbCoPanel = 3;
    }

    public static final class FalconConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM

        public static final double kP = 0.2;        // PID terms
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        // public static final double kFF = 0.056;     // value = 1023 * desired-percent-out / at-sensor-velocity-sensor-units-per-100ms
        public static final double kS = 0.02;
        public static final double kV = 0.000161;
    }

    public static final class ShooterConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM
        public static final double pidErrorTolerance = 200; // in RPM

        public static final double kP = 0.15;        // PID terms
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0;
        public static final double kV = 0.000164;

        public static final int[][] distanceFromTargetToRPMTable = {{5,3500},{10, 4350},{15,2450},{20,2500},{25,2550}};         // Values in feet, RPM
        public static final double shooterDefaultRPM = 2800;

    }

    public static final class UptakeConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM
    }

    public static final class TurretConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double maxOutputUncalibrated = 0.05;             // Max output power when turret is *not* calibrated
        public static final double maxOutputCalibrated = 0.3;               // Max output power when turret is calibrated
        public static final double ticksPerDegree = 2048.0/360.0 * 4*180/18;         // Divide by this to convert raw ticks to turrent degrees  (4:1 versaplanetary, plus chain gear ratio 18:180)
        public static final double rawVelocityToDegPerSec = 10.0 / ticksPerDegree;   // Multiply by this to convert raw velocity (ticksPer100ms) to degrees per second
        public static final double softLimitRev = -65.0;            // Reverse position soft limit, in turret degrees
        public static final double softLimitFwd = 80.0;            // Forward position soft limit, in turret degrees
        public static final double limitSwitchRev = -70.0;            // Position of physical reverse limit switch, in turret degrees
        public static final double limitSwitchFwd = 85.0;            // Position of physical forward limit switch, in turret degrees
        public static final double calSwitch = -4.0;            // Position of starting calibration limit switch, in turret degrees

        // Turn-to-angle constants
        public static final double kMaxTurnVelocity = 1150.0;        // Max turret velocity in degrees per second (extrapolted at 100% power)
        public static final double kClampTurnVelocity = 150.0;       // Max turret velocity allowed (for safety) in degrees per second
        public static final double kMaxTurnAcceleration = 200.0;     // Max acceleration in degrees per second^2
        public static final double tLagTurn = 0.020;          // Lag time to start/stop turning, or just one cycle forcast through scheduler
        public static double kITurnEnd = 0.005;         // Value of kI to use after the trapezoid profile is finished
        public static double kPTurn = 0.001;         // PID terms
        public static double kITurn = 0.000;
        public static double kDTurn = 0.000;
        public static double kSTurn = 0.012;       // Feed-forward terms
        public static double kVTurn = 0.00102;      
        public static double kATurn = 0.00010;
    }
    
    public static final class PiVisionConstants {
        public static final double width = 680;
        public static final double height = 480;
        public static final double endDistance = 10; // distance of the "sweet spot", in percent area of camera
        public static final double angleMultiplier = 1.064;
        public static final double vFov = 0.398983329; // half of fov
        public static final double hFov = 0.54630249; // half of fov

        // goal
        public static final double cameraHeight = 44.5; // inches from ground to center of camera
        public static final double targetHeight = 104; // inches from ground
        public static final double cameraAngle = 18; // angle

    }

}