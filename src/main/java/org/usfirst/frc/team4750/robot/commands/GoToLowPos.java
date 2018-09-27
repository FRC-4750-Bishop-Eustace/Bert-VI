package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;
import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command moves the elevator to the low position
 * 
 */
public class GoToLowPos extends Command {

	public GoToLowPos() {
		requires(Robot.elevator);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.elevator.getPosition() != 1) {
				Robot.elevator.setElevatorSpeed(RobotMap.ELEVATOR_DOWN_SPEED);	
		} else {
			Robot.elevator.stopElevator();
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
		Robot.elevator.stopElevator();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
