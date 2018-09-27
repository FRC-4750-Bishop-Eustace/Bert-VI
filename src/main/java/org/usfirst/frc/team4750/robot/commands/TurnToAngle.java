package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command handles the autonomous turning mode, keeping track of previous
 * turns to minimize error
 *
 */
public class TurnToAngle extends Command {

	// Create PID Controller
	public PIDController turnController;
	// Heading to turn to
	private float targetHeading;
	// Check if finished
	private boolean finished = false;
	// Cumulative actual heading
	private static float cumulativeActualHeading = 0.0f;
	// Calculate setpoint
	private float setpoint;

	// PID Values
	static final double P = 0.01; // 0.01
	static final double I = 0.003; // 0.003
	static final double D = 0.1; // 0.1
	static final double F = 0.6; // 0.6

	// Minimum error
	static final double toleranceDegrees = 2.0; // 2.0

	public TurnToAngle(float targetHeading) {
		// Get heading
		this.targetHeading = targetHeading;

		// System.out.println("TurnToAngle(): Contructor ran!");
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Reset the IMU
		Robot.imu.reset();
		// Set the setpoint to the target heading +/- the past error
		setpoint = ((float) Robot.imu.cumulativeCommandedHeading - cumulativeActualHeading) + targetHeading;
		// Add the target heading to the commanded heading
		// cumulativeCommandedHeading += targetHeading;
		Robot.imu.cumulativeCommandedHeading += targetHeading;
		// Initialize PID controller
		turnController = new PIDController(P, I, D, F, Robot.imu.ahrs, Robot.driveTrain);
		// Min and max angle to turn to
		turnController.setInputRange(-180.0f, 180.0f);
		// Max motor speed (0.6)
		turnController.setOutputRange(-1, 1); // (-1, 1)
		// Max error
		turnController.setAbsoluteTolerance(toleranceDegrees);
		turnController.setContinuous(true);
		// Set PID to turn to setpoint
		turnController.setSetpoint(setpoint);
		// Enable PID controller
		turnController.enable();

		// System.out.println("TurnToAngle(): Initialize ran!");
	}

	// Called repeatedly when this Command is scheduled to run
	/**
	 * read the current heading from the IMU store in a temporary variable compare
	 * target heading with what is compared in the new variable if target heading is
	 * < value in the new variable, make drive train turn left if target heading is
	 * > value in new variable, make drive train turn right
	 */
	@Override
	protected void execute() {
		// If the error is less than the tolerance, wait to make sure we aren't still
		// moving, then finish
		if (turnController.onTarget()) {
			Timer.delay(.05);
			if (turnController.onTarget()) {
				finished = true;
			} else {
				finished = false;
			}
		} else {
			finished = false;
		}

		// Call drive train
		Robot.driveTrain.pidTurn();

		// Wait for motor update
		Timer.delay(0.005);

		// System.out.println("TurnToAngle(): Execute");
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// Add the actual turn to account for any error
		cumulativeActualHeading += Robot.imu.getHeading();
		// Stop motors
		Robot.driveTrain.brake();
		// Disable PID controller
		turnController.disable();
		turnController.reset();
		Timer.delay(0.03);
		// System.out.println("TurnToAngle(): Should be at target!");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
		// System.out.println("TurnToAngle(): Interrupted ran!");
	}
}
