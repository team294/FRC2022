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
     * Ball locations within the robot.
     */
    public enum BallLocation {
        kUptake(0),
        kEject(1),
        kFeeder(2);

        public final int value;
        BallLocation(int value){
            this.value = value;
        }
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

    /**
     * Options to select driving coordinates.
     */
    public enum CoordType {
        kRelative(0),
        kAbsolute(1),
        kAbsoluteResetPose(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        CoordType(int value) { this.value = value; }
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop(0),
        kCoast(1),
        kBrake(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        StopType(int value) { this.value = value; }
    }

    public static final class RobotConstants {
        // Global constants go here
        public static final double temperatureCheck = 40; // in celsius
        
    }

    public static final class Ports {
        // CANbus addresses
        public static final int CANPneumaticHub = 1;
        public static final int CANPowerDistHub = 1;

        public static final int CANLeftDriveMotor1 = 11;
        public static final int CANLeftDriveMotor2 = 12;

        public static final int CANRightDriveMotor1 = 15;
        public static final int CANRightDriveMotor2 = 16;

        public static final int CANIntakeFront = 21;
        public static final int CANIntakeFrontTransfer = 22;    // Horizontal Omni wheels between intake and uptake
        public static final int CANIntakeRear = 23;

        public static final int CANUptake = 31;
        public static final int CANEject = 32;
        public static final int CANFeeder = 33;
        public static final int CANTurret = 34;
        public static final int CANShooter1 = 35;       // First shooter motor
        public static final int CANShooter2 = 36;       // Second shooter motor

        // Digital IO ports
        public static final int DIOTurretCalSwitch = 1;
        public static final int DIOFeederBallSensor = 2;
        public static final int DIOEjectBallSensor = 3;

        // I2C ports
        public static final int I2CcolorSensor = 0x52;       // According to REV docs, color sensor is at 0x52 = 82.  Rob had 39?

        // Pneumatic solenoid ports
        public static final int SolIntakeFrontFwd = 5;
        public static final int SolIntakeFrontRev = 6;
        public static final int SolIntakeRearFwd = 4;
        public static final int SolIntakeRearRev = 7;
        public static final int SolClimbLeft = 3;
        public static final int SolClimbRight = 2;
        public static final int SolClimbNA = 1;
    }
   
    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;
    }

    public static final class ShooterConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM
        public static final double pidErrorTolerance = 100; // in RPM

        public static final double kP = 0.02;        // PID terms
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.003;
        public static final double kV = 0.000154;

        public static final int[][] distanceFromTargetToRPMTable = {{5,3500},{10, 4350},{15,5200},{20,6050},{25,6900}};         // Values in feet, RPM
        public static final double shooterDefaultRPM = 2800;

    }

    public static final class UptakeConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM
    }

    public static final class IntakeConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
        public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM
        public static final double intakeDefaultPercent = 0.25;                     // Default on percentage
    }

    public static final class TurretConstants {
        public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
        public static final double maxOutputUncalibrated = 0.05;             // Max output power when turret is *not* calibrated
        public static final double maxOutputCalibrated = 0.3;               // Max output power when turret is calibrated
        public static final double ticksPerDegree = 2048.0/360.0 * 4*170/16;         // Divide by this to convert raw ticks to turrent degrees  (4:1 versaplanetary, plus chain gear ratio 18:180)
        public static final double rawVelocityToDegPerSec = 10.0 / ticksPerDegree;   // Multiply by this to convert raw velocity (ticksPer100ms) to degrees per second
        public static final double softLimitRev = -140.0;            // Reverse position soft limit, in turret degrees
        public static final double softLimitFwd = 140.0;            // Forward position soft limit, in turret degrees
        public static final double limitSwitchRev = -145.0;            // Position of physical reverse limit switch, in turret degrees
        public static final double limitSwitchFwd = 147.0;            // Position of physical forward limit switch, in turret degrees
        public static final double calSwitch = 0.0;            // Position of starting calibration limit switch, in turret degrees

        // Turn-to-angle constants
        public static final double kMaxTurnVelocity = 1150.0;        // Max turret velocity in degrees per second (extrapolted at 100% power)
        public static final double kClampTurnVelocity = 250.0;       // Max turret velocity allowed (for safety) in degrees per second
        public static final double kMaxTurnAcceleration = 500.0;     // Max acceleration in degrees per second^2
        public static final double kClampAccelShortTurn = 200.0;     // For short turns, Max acceleration in degrees per second^2
        public static final double kShortTurn = 20.0;           // Short turns are relative angle change less than +/- this variable, in degrees
        public static final double tLagTurn = 0.020;          // Lag time to start/stop turning, or just one cycle forcast through scheduler
        public static double kITurnEnd = 0.02;         // Value of kI to use after the trapezoid profile is finished
        public static double kPTurn = 0.001;         // PID terms  0.001
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

    public static final class LimeLightConstants {
        public static final double angleMultiplier = 1.064;

        // *******************************
        // The constants below are DEFAULT VALUES. Change these value in RobotPrefrences
        // for each robot, not in this code!
        // *******************************
        public static boolean takeSnapshots = true;
    }

    public static final class DriveConstants {

        // *******************************
        // The constants below apply to all robots and are not in RobotPreferences
        // *******************************

        public static final double compensationVoltage = 12.0; // voltage compensation on drive motors
        public static final double MAX_VOLTAGE_IN_TRAJECTORY = 10.0;

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;

        // *******************************
        // The constants below are DEFAULT VALUES. Change these value in RobotPrefrences
        // for each robot, not in this code!
        // *******************************

        public static double ticksPerInch = 1210.0; // TODO update for 2022

        // turnGyro constants
        public static double kMaxAngularVelocity = 1125; // degrees per second  // TODO update for 2022
        public static double kMaxAngularAcceleration = 200; // degrees per second per second  // TODO update for 2022
        public static double kVAngular = 0.00108;   // TODO update for 2022
        public static double kAAngular = 0.00015;   // TODO update for 2022
        public static double kSAngular = 0.01;      // TODO update for 2022
        public static double kPAngular = 0.001;     // TODO update for 2022
        public static double kDAngular = 0;
        public static double kIAngular = 0.015;    // TODO update for 2022
        public static double tLagAngular = 0.020;          // Lag time to start/stop turning, or just one cycle forcast through scheduler
        public static final double maxSecondsForTurnGyro = 2.0; // max time to wait for turn gyro. use this in commands to timeout

        public static final double wheelInchesToGyroDegrees = 4.205; // converts from inches traveled by the wheels when spinning in place to degrees turned // TODO update for 2022

        // DriveStraight constants
        public static double kMaxSpeedMetersPerSecond = 5.22;   // TODO update for 2022
        public static double kMaxAccelerationMetersPerSecondSquared = 3.8;   // TODO update for 2022
        public static double kVLinear = 0.187;  // TODO update for 2022
        public static double kALinear = 0.025;  // TODO update for 2022
        public static double kSLinear = 0.024;  // TODO update for 2022
        public static double kPLinear = 0.280;  // TODO update for 2022
        public static double kILinear = 0;
        public static double kDLinear = 0; 
        public static double kAngLinear = 0.030; // TODO update for 2022

        // Trajectory generation constants
        public static double kS = kSLinear * compensationVoltage; 
        public static double kV = kVLinear * compensationVoltage; 
        public static double kA = kALinear * compensationVoltage; 

        public static double TRACK_WIDTH = 0.71;   // in meters     // TODO update for 2022

        public static void updateDerivedConstants() {
            kS = kSLinear * compensationVoltage; 
            kV = kVLinear * compensationVoltage; 
            kA = kALinear * compensationVoltage; 
        }
    }

}