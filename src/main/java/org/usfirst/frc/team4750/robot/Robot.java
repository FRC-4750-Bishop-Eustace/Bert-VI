package org.usfirst.frc.team4750.robot;

import java.awt.Point;
import java.util.HashMap;

import org.usfirst.frc.team4750.robot.commands.AStarAuton;
import org.usfirst.frc.team4750.robot.commands.LFromL;
import org.usfirst.frc.team4750.robot.commands.LFromM;
import org.usfirst.frc.team4750.robot.commands.LScaleFromL;
import org.usfirst.frc.team4750.robot.commands.LeftBaseline;
import org.usfirst.frc.team4750.robot.commands.RFromM;
import org.usfirst.frc.team4750.robot.commands.RFromR;
import org.usfirst.frc.team4750.robot.commands.RScaleFromR;
import org.usfirst.frc.team4750.robot.commands.Release;
import org.usfirst.frc.team4750.robot.commands.Reset;
import org.usfirst.frc.team4750.robot.commands.RightBaseline;
import org.usfirst.frc.team4750.robot.commands.SwitchElevatorMode;
import org.usfirst.frc.team4750.robot.pathfinding.AStar;
import org.usfirst.frc.team4750.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4750.robot.subsystems.Elevator;
import org.usfirst.frc.team4750.robot.subsystems.Encoders;
import org.usfirst.frc.team4750.robot.subsystems.Grabber;
import org.usfirst.frc.team4750.robot.subsystems.IMU;
import org.usfirst.frc.team4750.robot.subsystems.Lifter;
import org.usfirst.frc.team4750.robot.subsystems.Limelight;
import org.usfirst.frc.team4750.robot.subsystems.Ultrasonics;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// Initialize subsystems
	public static final StraightDriveSource averageEncoderPIDSource = new StraightDriveSource();
	public static final StraightDrivePIDRelay straightDrivePIDRelay = new StraightDrivePIDRelay();
	public static final IMU imu = new IMU();
	public static final DriveTrain driveTrain = new DriveTrain(RobotMap.FRONT_LEFT_MOTOR_ID,
			RobotMap.FRONT_RIGHT_MOTOR_ID, RobotMap.LEFT_MOTOR_ID, RobotMap.RIGHT_MOTOR_ID, RobotMap.BACK_LEFT_MOTOR_ID,
			RobotMap.BACK_RIGHT_MOTOR_ID);
	public static final Ultrasonics ultrasonic = new Ultrasonics();
	public static final Encoders encoders = new Encoders();
	public static final Limelight limelight = new Limelight();
	public static final Elevator elevator = new Elevator();
	public static final Lifter lifter = new Lifter();
	public static final Grabber grabber = new Grabber();
	public static OI oi;

	// Autonomous data
	public static String gameData;
	Command autonomousCommand;

	// Start position chooser
	static SendableChooser<String> startPositionChooser = new SendableChooser<>();

	// Goal position chooser
	static GoalChooser objectiveLL = new GoalChooser();
	static GoalChooser objectiveLR = new GoalChooser();
	static GoalChooser objectiveRL = new GoalChooser();
	static GoalChooser objectiveRR = new GoalChooser();

	// Final orientation chooser
	static OrientationChooser orientationLL = new OrientationChooser();
	static OrientationChooser orientationLR = new OrientationChooser();
	static OrientationChooser orientationRL = new OrientationChooser();
	static OrientationChooser orientationRR = new OrientationChooser();

	// Starting points hash map
	public static HashMap<String, Point> startPoints;

	// Goal points hash map
	public static HashMap<String, Point> goalPoints;

	// Final goal position
	Point goalPosition;

	// Final orientation
	static String orientation;

	// Reset command
	Command reset = new Reset();

	// Switch elevator mode command
	Command switchElevatorMode = new SwitchElevatorMode();

	// Release piston command
	Command release = new Release();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();

		// Initialize starting points hash maps
		startPoints = new HashMap<String, Point>();
		startPoints.put("m", new Point(14, 2));
		startPoints.put("l", new Point(24, 2));
		startPoints.put("r", new Point(4, 2));

		// Initialize goal points hash maps
		goalPoints = new HashMap<String, Point>();
		goalPoints.put("fls", new Point(20, 12));
		goalPoints.put("frs", new Point(8, 12));
		goalPoints.put("ls", new Point(21, 15));
		goalPoints.put("rs", new Point(7, 15));
		goalPoints.put("bls", new Point(20, 19));
		goalPoints.put("brs", new Point(8, 19));
		goalPoints.put("flsc", new Point(23, 25));
		goalPoints.put("frsc", new Point(5, 25));
		goalPoints.put("lsc", new Point(23, 27));
		goalPoints.put("rsc", new Point(7, 27));

		// Start position chooser
		startPositionChooser.addDefault("Middle", "m");
		startPositionChooser.addObject("Left", "l");
		startPositionChooser.addObject("Right", "r");
		SmartDashboard.putData("Start Position", startPositionChooser);

		// Goal choosers
		SmartDashboard.putData("LL Objective", objectiveLL.getChooser());
		SmartDashboard.putData("LR Objective", objectiveLR.getChooser());
		SmartDashboard.putData("RL Objective", objectiveRL.getChooser());
		SmartDashboard.putData("RR Objective", objectiveRR.getChooser());

		// Final orientation choosers
		SmartDashboard.putData("LL Orientation", orientationLL.getChooser());
		SmartDashboard.putData("LR Orientation", orientationLR.getChooser());
		SmartDashboard.putData("RL Orientation", orientationRL.getChooser());
		SmartDashboard.putData("RR Orientation", orientationRR.getChooser());

		// Reset sensors
		reset.start();

		// Put reset command on dashboard
		SmartDashboard.putData("Reset", reset);

		// Put release command on dashboard
		SmartDashboard.putData("Release", release);

		// Put switch elevator mode command on dashboard
		SmartDashboard.putData("Switch Elevator Mode", switchElevatorMode);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		// Do nothing
	}

	@Override
	public void disabledPeriodic() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// Create planner
		AStar pathPlanner = new AStar();
		try { // Try to get a route
			autonomousCommand = new AStarAuton(pathPlanner.getPath(getStartPoint(getStart()), getGoalPoint(getGoal())));
		} catch (Exception e) { // Failed route, print the stack trace and run regular 
			System.out.println("PathPlanning failed. Running regular autonomous: ");
			e.printStackTrace();
			if (gameData == null) {
				gameData = DriverStation.getInstance().getGameSpecificMessage();
			} else {
				if (getStart().equalsIgnoreCase("L")) {
					if (gameData.charAt(0) == 'L') {
						autonomousCommand = new LFromL();
					} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
						autonomousCommand = new LScaleFromL();
					} else {
						autonomousCommand = new LeftBaseline();
					}
				} else if (getStart().equalsIgnoreCase("R")) {
					if (gameData.charAt(0) == 'R') {
						autonomousCommand = new RFromR();
					} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
						autonomousCommand = new RScaleFromR();
					} else {
						autonomousCommand = new RightBaseline();
					}
				} else if (getStart().equalsIgnoreCase("M")) {
					if (gameData.charAt(0) == 'R') {
						autonomousCommand = new RFromM();
					} else if (gameData.charAt(0) == 'L') {
						autonomousCommand = new LFromM();
					}
				}
			}
		}

		// Schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// Set limelight mode to camera mode for dashboard
		limelight.setCameraMode("Camera");

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		// Do nothing
	}

	/**
	 * This method returns the selected objective for whatever order the plates are
	 * in
	 * 
	 * @return goal position
	 */
	public static String getGoal() {
		// Get game data if not already gotten
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		// Make sure game data is not null
		if (gameData == null) {
			return null;
		}

		// Get the switch and scale position
		char switchPos = gameData.charAt(0);
		char scalePos = gameData.charAt(1);

		// Check which position the switch and scale are at and get the goal point
		// chosen from the dashboard
		if (switchPos == 'L' && scalePos == 'L') {
			return objectiveLL.getSelected();
		} else if (switchPos == 'L' && scalePos == 'R') {
			return objectiveLR.getSelected();
		} else if (switchPos == 'R' && scalePos == 'L') {
			return objectiveRL.getSelected();
		} else if (switchPos == 'R' && scalePos == 'R') {
			return objectiveRR.getSelected();
		}
		return null;
	}

	/**
	 * This method searches the hash map for the position to go to and returns the
	 * point of that position
	 * 
	 * @param position
	 *            - normally gotten from the getGoal() method
	 * @return goal position point
	 */
	public Point getGoalPoint(String position) {
		return goalPoints.get(position);

	}

	/**
	 * This method returns the currently selected start position
	 * 
	 * @return start position
	 */
	public static String getStart() {
		return startPositionChooser.getSelected();
	}

	/**
	 * This method searches the hash map for the starting position to and returns
	 * the point of that position
	 * 
	 * @param position
	 *            - normally gotten from the getStart() method
	 * @return start position point
	 */
	public Point getStartPoint(String position) {
		return startPoints.get(position);
	}

	/**
	 * This method returns the selected orientation for whatever order the plates
	 * are in
	 * 
	 * @return goal orientation
	 */
	public static double getGoalOrientation() {
		// Get game data if not already gotten
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		// Make sure game data is not null
		if (gameData == null) {
			return 0;
		}

		// Get the switch and scale position
		char switchPos = gameData.charAt(0);
		char scalePos = gameData.charAt(1);

		// Check which position the switch and scale are at and get the final
		// orientation chosen from the dashboard
		if (switchPos == 'L' && scalePos == 'L') {
			orientation = orientationLL.getSelected();
		} else if (switchPos == 'L' && scalePos == 'R') {
			orientation = orientationLR.getSelected();
		} else if (switchPos == 'R' && scalePos == 'L') {
			orientation = orientationRL.getSelected();
		} else if (switchPos == 'R' && scalePos == 'R') {
			orientation = orientationRR.getSelected();
		}

		// Return the final angle
		if (orientation.equalsIgnoreCase("f")) {
			return 0;
		} else if (orientation.equalsIgnoreCase("l")) {
			return -90;
		} else if (orientation.equalsIgnoreCase("r")) {
			return 90;
		} else if (orientation.equalsIgnoreCase("b")) {
			return 180;
		}
		return 0;
	}
}
