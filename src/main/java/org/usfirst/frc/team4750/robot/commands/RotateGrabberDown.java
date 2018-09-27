package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;
import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command rotates the grabber down
 * 
 */
public class RotateGrabberDown extends Command {

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (!Robot.grabber.getLow()) {
			Robot.grabber.rotateGrabber(RobotMap.GRABBER_DOWN_SPEED);
		} else {
			Robot.grabber.rotateGrabber(0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.grabber.rotateGrabber(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
