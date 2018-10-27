package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;
import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This class rotates the grabber up
 * 
 */
public class RotateGrabberUp extends Command {
	
	boolean timed;
	
	boolean finished = false;
	
	public RotateGrabberUp(boolean timed) {
		this.timed = timed;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(Robot.grabber.getPiston()) return;
		if(!timed) {
			if (!Robot.grabber.getHigh()) {
				Robot.grabber.rotateGrabber(RobotMap.GRABBER_UP_SPEED);
			} else {
				Robot.grabber.rotateGrabber(0);
			}	
		}else {
			Robot.grabber.rotateGrabber(RobotMap.GRABBER_UP_SPEED);
			Timer.delay(1);
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
		Robot.grabber.rotateGrabber(0.1);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
