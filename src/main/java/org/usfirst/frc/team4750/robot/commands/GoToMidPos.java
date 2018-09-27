package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;
import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command moves the elevator to the middle position
 * 
 */
public class GoToMidPos extends Command {

	public GoToMidPos() {
		requires(Robot.elevator);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.elevator.getPosition() != 2) {
			if (Robot.elevator.getPosition() > 2) {
				Robot.elevator.setElevatorSpeed(RobotMap.ELEVATOR_DOWN_TO_MID_SPEED);
			} else if (Robot.elevator.getPosition() < 2) {
				Robot.elevator.setElevatorSpeed(RobotMap.ELEVATOR_UP_SPEED);
			}
		} else {
			Robot.elevator.stallElevator();
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
