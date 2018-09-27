package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToRaise extends Command {

	// Distance from switch
	double distance = 11;

	// Drive speed
	double speed = 0.4;
	
	//
	double tolerance = 0.1;

	// Check if finished
	boolean finished = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.ultrasonic.getInches() > (distance + tolerance)) {
			Robot.driveTrain.setDriveSpeed(speed);
			finished = false;
		} else if (Robot.ultrasonic.getInches() < (distance - tolerance)) {
			Robot.driveTrain.setDriveSpeed(-speed);
			finished = false;
		} else {
			Robot.driveTrain.brake();
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
		Robot.driveTrain.brake();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
