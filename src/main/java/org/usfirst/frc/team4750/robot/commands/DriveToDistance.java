package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command is a PID controlled drive-to-distance command using encoders
 *
 */
public class DriveToDistance extends Command {

	// Create PID Controller
	PIDController driveController;
	// Distance to drive to
	private float targetDistance;
	// Check if finished
	private boolean finished = false;

	// PID Values
	static final double P = 0.06; // 0.06
	static double I = 0.004; // 0.004
	static final double D = 0.8; // 0.8
	static final double F = 1.0; // 1.0

	// Minimum error
	static final double tolerance = 1.0; // 1.0

	// Takes in distance in inches or feet
	public DriveToDistance(float distance, boolean feet) {
		// If feet is false, then the input is in inches. If not, convert feet to
		// inches.
		if (!feet) {
			this.targetDistance = distance;
		} else {
			// Convert feet to inches
			this.targetDistance = distance * 12;
		}

		// System.out.println("DriveToDistance(): Constructor ran!");
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Reset encoders before driving
		Robot.encoders.reset();
		// Initialize PID controller
		driveController = new PIDController(P, I, D, F, Robot.averageEncoderPIDSource, Robot.driveTrain);
		// Max motor speed
		driveController.setOutputRange(-1, 1); // (-1, 1)
		// Max error
		driveController.setAbsoluteTolerance(tolerance);
		// Set PID to turn to setpoint
		driveController.setSetpoint(targetDistance);
		// Enable PID controller
		driveController.enable();

		// System.out.println("DriveToDistance(): Initialize ran!");
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
		if (driveController.onTarget()) {
			Timer.delay(.1);
			if (driveController.onTarget()) {
				finished = true;
			} else {
				finished = false;
			}
		} else {
			finished = false;
		}

		// Call drive train
		Robot.driveTrain.pidDrive();

		// Wait for motor update
		Timer.delay(0.005);

		System.out.println("DriveToDistance(): Execute ran!");
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// Stop motors
		Robot.driveTrain.brake();
		// Disable PID controller
		driveController.disable();
		System.out.println("DriveToDistance(): Should be at target!");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
		// System.out.println("DriveToDistance(): Interrupted ran!");
	}
}
