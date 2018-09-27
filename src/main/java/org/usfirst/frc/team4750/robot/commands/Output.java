package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This command handles all sensor outputs to the smart dashboard
 * 
 */
public class Output extends Command {
	
	public Output() {
		requires(Robot.imu);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Output encoders to dashboard
		SmartDashboard.putData("Left Encoder", Robot.encoders.leftEncoder);
		SmartDashboard.putData("Right Encoder", Robot.encoders.rightEncoder);

		// Output IMU to dashboard
		SmartDashboard.putData("IMU", Robot.imu.ahrs);

		// Output ultrasonic sensor to dashboard
		SmartDashboard.putData("Ultrasonic", Robot.ultrasonic.ultrasonic);

		// Output elevator mode
		SmartDashboard.putString("Elevator Mode", Robot.elevator.getMode());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
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
