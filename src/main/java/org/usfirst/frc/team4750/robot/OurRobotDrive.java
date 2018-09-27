package org.usfirst.frc.team4750.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This class acts as a custom DifferentialDrive controller to adjust values and
 * allow speed ("gear") shifting
 *
 */
public class OurRobotDrive extends DifferentialDrive {

	// Constant to protect from brownouts
	double constant = 0.85;

	// Takes in two drive motors (or speed controller groups)
	public OurRobotDrive(SpeedController leftMotor, SpeedController rightMotor) {
		super(leftMotor, rightMotor);
	}

	// Supers the tank drive, protecting the motors from browning out
	@Override
	public void tankDrive(double leftSpeed, double rightSpeed) {
		super.tankDrive(brownOut(leftSpeed), brownOut(rightSpeed));
	}

	// Supers the arcade drive, protecting the motors from browning out
	@Override
	public void arcadeDrive(double forwardSpeed, double turnSpeed) {
		super.arcadeDrive(brownOut(forwardSpeed), brownOut(turnSpeed));
	}

	// Protects the motors from brownouts
	private double brownOut(double value) {
		return value * constant;
	}

}