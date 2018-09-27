package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.RobotMap;
import org.usfirst.frc.team4750.robot.commands.RunLifter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subsystem controls the lifting and switching of the elevator mechanism
 *
 */
public class Lifter extends Subsystem {

	// Motors
	WPI_TalonSRX lifter;
	
	// Pistons
	Solenoid releasePiston;

	public Lifter() {
		// Initialize motors
		lifter = new WPI_TalonSRX(RobotMap.LIFTER_MOTOR_ID);
		
		// Initialize pistons
		releasePiston = new Solenoid(RobotMap.RELEASE_PISTON_PORT);
	}

	/**
	 * Sets lifter motor to speed
	 * 
	 * @param speed
	 */
	public void setLifterSpeed(double speed) {
		lifter.set(speed);
	}

	/**
	 * Stops lifter motor
	 */
	public void stopLifter() {
		lifter.set(0);
	}
	
	/**
	 * Toggle the release piston to drop the grabber
	 */
	public void releasePiston() {
		releasePiston.set(!releasePiston.get());
	}

	/**
	 * Sets command to run via joysticks automatically
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new RunLifter());
	}
}
