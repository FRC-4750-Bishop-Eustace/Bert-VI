/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.OI;
import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command calls the drive train to run the default joystick drive. When
 * button two on the left drive stick is held down, it will switch from single
 * stick arcade drive to dual stick tank drive
 * 
 */
public class TankDrive extends Command {

	static final double offBalanceThreshold = 5;
	static final double onBalanceThreshold = 5;
	static final double P = 0.1;

	public TankDrive() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		requires(Robot.driveTrain);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Get the current pitch of the robot
		double pitch = Robot.imu.getPitch();

		// // Check if we are tilting past the threshold, try to correct
		// if (Math.abs(pitch) >= offBalanceThreshold && pitch > 0) {
		// 	if (Math.abs(pitch) >= onBalanceThreshold) {
		// 		pitch = Robot.imu.getPitch();
		// 		Robot.driveTrain.setDriveSpeed(P * pitch);
		// 	}
		// } else if (Math.abs(pitch) >= offBalanceThreshold && pitch < 0) {
		// 	if (Math.abs(pitch) >= onBalanceThreshold) {
		// 		pitch = Robot.imu.getPitch();
		// 		Robot.driveTrain.setDriveSpeed(P * pitch);
		// 	}
		// }

		if (OI.leftDriveStick.getRawButton(2)) {
			// Call dual joystick drive and pass in both joysticks
			Robot.driveTrain.dualDrive(OI.leftDriveStick, OI.rightDriveStick);
		} else {
			// Call single joystick drive and pass in the right joystick
			Robot.driveTrain.singleDrive(OI.rightDriveStick);
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
		// Do nothing
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// Do nothing
	}
}
