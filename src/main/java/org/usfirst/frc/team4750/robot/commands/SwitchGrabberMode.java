package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command opens/closes the grabber piston
 *
 */
public class SwitchGrabberMode extends Command {

	// Check if finished
	boolean finished = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.grabber.switchPistons();
		finished = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// Do nothing
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// Do nothing
	}
}
