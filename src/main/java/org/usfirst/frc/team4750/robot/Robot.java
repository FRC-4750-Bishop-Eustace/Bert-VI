package org.usfirst.frc.team4750.robot;

import java.awt.Point;
import java.io.File;
import java.util.HashMap;

import org.usfirst.frc.team4750.robot.commands.AStarAuton;
import org.usfirst.frc.team4750.robot.commands.LFromL;
import org.usfirst.frc.team4750.robot.commands.LFromM;
import org.usfirst.frc.team4750.robot.commands.LScaleFromL;
import org.usfirst.frc.team4750.robot.commands.LeftBaseline;
import org.usfirst.frc.team4750.robot.commands.Lift;
import org.usfirst.frc.team4750.robot.commands.RFromM;
import org.usfirst.frc.team4750.robot.commands.RFromR;
import org.usfirst.frc.team4750.robot.commands.RScaleFromR;
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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

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

	File leftFile;
	File rightFile;
	Trajectory leftTrajectory;
	Trajectory rightTrajectory;
	public static DistanceFollower left;
	public static DistanceFollower right;
	public static Notifier autoNotifier;
	double wheelbase = 1.9791667; // Feet
	double wheelDiameter = 0.5; // Feet
	double maxVel = 9.256463818; // Feet/second
	double maxAccl = 9.84251969; // Feet/second^2
	double maxJerk = 196.850394; // Feet/second^3

	// Start position chooser
	static SendableChooser<String> startPositionChooser = new SendableChooser<>();

	// Starting points hash map
	public static HashMap<String, Point> startPoints;

	// Goal points hash map
	public static HashMap<String, Point> goalPoints;

	// Final goal position
	Point goalPosition;

	// Final orientation
	static String orientation;

	Command reset = new Reset();

	// Switch elevator mode command
	Command switchElevatorMode = new SwitchElevatorMode();

	// Release piston command
	Command lift = new Lift();

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

		// Reset sensors
		reset.start();

		// Put reset command on dashboard
		SmartDashboard.putData("Reset", reset);

		// Put release command on dashboard
		SmartDashboard.putData("Lift", lift);

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
		if (getStart().equalsIgnoreCase("M")) {
			if (gameData.charAt(0) == 'L') {
				autonomousCommand = new LFromM();
			} else if (gameData.charAt(0) == 'R') {
				autonomousCommand = new RFromM();
			}
		} else if (getStart().equalsIgnoreCase("L")) {
			if (gameData.charAt(0) == 'L') {
				autonomousCommand = new LFromL();
			} else if (gameData.charAt(0) == 'R') {
				autonomousCommand = new LeftBaseline();
			}
		} else if (getStart().equalsIgnoreCase("R")) {
			if (gameData.charAt(0) == 'L') {
				autonomousCommand = new RightBaseline();
			} else if (gameData.charAt(0) == 'R') {
				autonomousCommand = new RFromR();
			}
		}

		leftFile = new File("/home/lvuser/paths/Test_left_Jaci.csv");
		rightFile = new File("/home/lvuser/paths/Test_right_Jaci.csv");
		leftTrajectory = Pathfinder.readFromCSV(leftFile);
		rightTrajectory = Pathfinder.readFromCSV(rightFile);
		left = new DistanceFollower(leftTrajectory);
		right = new DistanceFollower(rightTrajectory);
		left.configurePIDVA(1.0, 0.0, 0.6, 1 / maxVel, 0);
		right.configurePIDVA(1.0, 0.0, 0.3, 1 / maxVel, 0);

		//autonomousCommand = new RScaleFromR();

		//autoNotifier = new Notifier(new RunAuton());
		//autoNotifier.startPeriodic(0.05);

		// // Create planner
		// AStar pathPlanner = new AStar();
		// try { // Try to get a route
		// 	autonomousCommand = new AStarAuton(pathPlanner.getPath(getStartPoint(getStart()), getGoalPoint(getGoal())));
		// } catch (Exception e) { // Failed route, print the stack trace and run regular 
		// 	System.out.println("PathPlanning failed. Running regular autonomous: ");
		// 	e.printStackTrace();
		// 	if (gameData == null) {
		// 		gameData = DriverStation.getInstance().getGameSpecificMessage();
		// 	} else {
		// 		if (getStart().equalsIgnoreCase("L")) {
		// 			if (gameData.charAt(0) == 'L') {
		// 				autonomousCommand = new LFromL();
		// 			} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
		// 				autonomousCommand = new LScaleFromL();
		// 			} else {
		// 				autonomousCommand = new LeftBaseline();
		// 			}
		// 		} else if (getStart().equalsIgnoreCase("R")) {
		// 			if (gameData.charAt(0) == 'R') {
		// 				autonomousCommand = new RFromR();
		// 			} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
		// 				autonomousCommand = new RScaleFromR();
		// 			} else {
		// 				autonomousCommand = new RightBaseline();
		// 			}
		// 		} else if (getStart().equalsIgnoreCase("M")) {
		// 			if (gameData.charAt(0) == 'R') {
		// 				autonomousCommand = new RFromM();
		// 			} else if (gameData.charAt(0) == 'L') {
		// 				autonomousCommand = new LFromM();
		// 			}
		// 		}
		// 	}
		// }
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
		System.out.println("Left encoder raw: " + encoders.getLeftCount());
		System.out.println("Right encoder raw: " + encoders.getRightCount());
		System.out.println("Left encoder inches: " + encoders.getLeftDistanceInches());
		System.out.println("Right encoder inches: " + encoders.getRightDistanceInches());
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
		return "l";
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

	class RunAuton implements Runnable {
		@Override
		public void run() {
			System.out.println("Left finished: " + Robot.left.isFinished());
			System.out.println("Right finished: " + Robot.right.isFinished());
			if (!Robot.left.isFinished() || !Robot.right.isFinished()) {
				double l = Robot.left.calculate(Robot.encoders.getLeftDistanceFeet());
				double r = Robot.right.calculate(Robot.encoders.getRightDistanceFeet());

				double gyro_heading = Robot.imu.getAngle();
				double desired_heading = Pathfinder.r2d(Robot.left.getHeading());

				double angleDifference = 0.8 * (-1.0 / 80.0)
						* Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
				double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

				System.out.println(l * 0.1);

				Robot.driveTrain.tankDrive(l * 0.1, r * 0.1);
			} else {
				Robot.driveTrain.brake();
				Robot.autoNotifier.stop();
			}
		}
	}
}