package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This class handle the default control of the grabber
 *
 */
public class RunGrabber extends Command {

	// Drive speeds
	double forwardSpeed, rotateSpeed;

	// Timer boolean
	boolean timed;

	// Check if finished
	boolean finished = false;

	public RunGrabber(double forwardSpeed, double rotateSpeed, boolean timed) {
		this.forwardSpeed = -forwardSpeed;
		this.rotateSpeed = -rotateSpeed;
		this.timed = timed;
		requires(Robot.grabber);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (!timed) {
			Robot.grabber.armDrive(forwardSpeed, rotateSpeed);
		} else {
			Robot.grabber.armDrive(forwardSpeed, rotateSpeed);
			Timer.delay(1.5);
			finished = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.grabber.armDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
