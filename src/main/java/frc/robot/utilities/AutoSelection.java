package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.commandGroups.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import frc.robot.Constants.CoordType;
// import frc.robot.Constants.SearchType;
import frc.robot.Constants.StopType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int TAXI = 0;
	public static final int SHOOT_TAXI = 1;
	public static final int TWO_BALL = 2;
	
	private TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, FileLog log) {
		this.trajectoryCache = trajectoryCache;

		// auto selections
		autoChooser.setDefaultOption("Two Ball", TWO_BALL);
		autoChooser.addOption("Shoot then Taxi", SHOOT_TAXI);
		autoChooser.addOption("Taxi", TAXI);
	
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
	public Command getAutoCommand(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, LimeLight limeLight, FileLog log) {
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

		if (autoPlan == SHOOT_TAXI) {
			log.writeLogEcho(true, "AutoSelect", "run AutoShootTaxi");
			autonomousCommand = new AutoShootTaxi(waitTime, driveTrain, shooter, feeder, intake, uptake, limeLight, log);
		}
		
		if (autoPlan == TAXI) {
			log.writeLogEcho(true, "AutoSelect", "run AutoTaxi");
			autonomousCommand = new AutoTaxi(waitTime, limeLight, driveTrain, log);
		}

		if (autoPlan == TWO_BALL) {
			log.writeLogEcho(true, "AutoSelect", "run AutoTwoBall");
			autonomousCommand = new AutoTwoBall(waitTime, driveTrain, shooter, feeder, intake, uptake, limeLight, log);
		}

		if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
