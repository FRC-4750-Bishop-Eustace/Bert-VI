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
	double constant = 0.9;
	double midConstant = 0.7;
	double highConstant = 0.5;

	// Takes in two drive motors (or speed controller groups)
	public OurRobotDrive(SpeedController leftMotor, SpeedController rightMotor) {
		super(leftMotor, rightMotor);
	}

	// Supers the tank drive, protecting the motors from browning out
	@Override
	public void tankDrive(double leftSpeed, double rightSpeed) {
		super.tankDrive(gear(leftSpeed), gear(rightSpeed));
	}

	// Supers the arcade drive, protecting the motors from browning out
	@Override
	public void arcadeDrive(double forwardSpeed, double turnSpeed) {
		super.arcadeDrive(gear(forwardSpeed), gear(turnSpeed));
	}

	// Protects the motors from brownouts
	private double gear(double value) {
		if(Robot.elevator.getPosition() == 1) {
			return value * constant;
		}else if(Robot.elevator.getPosition() == 2) {
			return value * midConstant;
		}else if(Robot.elevator.getPosition() == 3) {
			return value * highConstant;
		}else {
			return value * midConstant;
		}
	}

}