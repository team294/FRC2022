package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
// import frc.robot.Constants.CoordType;
// import frc.robot.Constants.SearchType;
// import frc.robot.Constants.StopType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int OPPONENT_TRENCH_PICKUP = 0;
	public static final int SHOOT_BACKUP = 1;
	public static final int TRUSS_PICKUP = 2;
	// Add more auto options here (step 1 of 3)...
	
	private TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, FileLog log) {
		this.trajectoryCache = trajectoryCache;

		// auto selections
		autoChooser.setDefaultOption("ShootBackup", SHOOT_BACKUP);
		autoChooser.addOption("OpponentTrenchPickup", OPPONENT_TRENCH_PICKUP);
		autoChooser.addOption("TrussPickup", TRUSS_PICKUP);
		// Add more auto options here (step 2 of 3)...
	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param shooter
	 * @param feeder
	 * @param hopper
	 * @param intake
	 * @param limeLightGoal
	 * @param log        The filelog to write the logs to
	 * @param led
	 * @return the command to run
	 */
	public Command getAutoCommand(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, 
			// LimeLightGoal limeLightGoal, LimeLightBall limeLightBall, LED led, 
			FileLog log) {
		Command autonomousCommand = null;
		Trajectory trajectory;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();

		boolean useVision = SmartDashboard.getBoolean("Autonomous use vision", false);
		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		// if (autoPlan == OPPONENT_TRENCH_PICKUP && trajectoryCache.cache[TrajectoryType.opponentTrenchPickup.value] != null) {
		// 	log.writeLogEcho(true, "AutoSelect", "run TrenchFromRight");
		// 	trajectory = trajectoryCache.cache[TrajectoryType.opponentTrenchPickup.value];
		// 	autonomousCommand = new AutoOpponentTrenchPickup(waitTime, useVision, trajectory, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		// }

		// if (autoPlan == SHOOT_BACKUP) {
		// 	log.writeLogEcho(true, "AutoSelect", "run ShootBackup");
		// 	autonomousCommand = new AutoShootBackup(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		// }

		// if (autoPlan == TRUSS_PICKUP) {
		// 	log.writeLogEcho(true, "AutoSelect", "run TrussPickup");
		// 	autonomousCommand = new AutoTrussPickup(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		// }

		// if (autoPlan == SLALOM_PATH){
		// 	log.writeLogEcho(true, "AutoSelect", "run SlalomPath");
		// 	autonomousCommand = new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.slalom.value], 
		// 		driveTrain, log);
		// }

		// if (autoPlan == RED_A && trajectoryCache.cache[TrajectoryType.galacticRedA.value] != null) {
		// 	log.writeLogEcho(true, "AutoSelect", "run Galactic Red A");
		// 	autonomousCommand = new AutoGalacticSearch(SearchType.kRedA, trajectoryCache, driveTrain, log, intake);
		// }

		// Add more auto options here (step 3 of 3)...
		
		if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
