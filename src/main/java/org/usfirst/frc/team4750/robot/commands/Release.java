package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command releases the drop wire for the crate grab when lifting
 *
 */
public class Release extends Command {

	boolean finished = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Make sure we are in end game, we only want to be able to run it if we are
		if (Timer.getMatchTime() > 30) {
			finished = true;
		} else {
			Robot.lifter.releasePiston();
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
		// Do nothing
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// Do nothing
	}
}
