package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class is the interface to the ultrasonic sensors (SR04)
 * 
 */
public class Ultrasonics extends Subsystem {

	// Sensors
	public Ultrasonic ultrasonic;

	public Ultrasonics() {
		// Create an ultrasonic sensor with pingChannel = trigger, echoChannel = echo
		ultrasonic = new Ultrasonic(RobotMap.ULTRASONIC_TRIGGER, RobotMap.ULTRASONIC_ECHO);
		// Automatically send and receive pulses
		ultrasonic.setAutomaticMode(true);
	}

	/**
	 * Use this to get the range in inches
	 * 
	 * @return inches from object
	 */
	public double getInches() {
		return ultrasonic.getRangeInches();
	}

	/**
	 * Use this to get the range in feet
	 * 
	 * @return feet from object
	 */
	public double getFeet() {
		return ultrasonic.getRangeInches() / 12;
	}

	@Override
	public void initDefaultCommand() {
		// Do nothing
	}
}
