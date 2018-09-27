package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subsystem controls the lifting and switching of the elevator mechanism
 *
 */
public class Elevator extends Subsystem {

	// Motors
	VictorSP leftElevator;
	VictorSP rightElevator;

	// Pistons
	Solenoid elevatorPiston;

	// Reed switches
	public DigitalInput lowPos;
	public DigitalInput midPos;
	public DigitalInput highPos;

	// Elevator position, 1 = low, 2 = mid, 3 = high
	public int position = 1;

	// Elevator mode
	String mode = "Elevator";

	public Elevator() {
		// Initialize motors
		leftElevator = new VictorSP(RobotMap.LEFT_ELEVATOR_MOTOR_PORT);
		rightElevator = new VictorSP(RobotMap.RIGHT_ELEVATOR_MOTOR_PORT);

		// Initialize pistons
		elevatorPiston = new Solenoid(RobotMap.ELEVATOR_PISTON_PORT);

		// Initialize reed switches
		lowPos = new DigitalInput(RobotMap.LOW_POSITION_ELEVATOR_PORT);
		midPos = new DigitalInput(RobotMap.MID_POSITION_ELEVATOR_PORT);
		highPos = new DigitalInput(RobotMap.HIGH_POSITION_ELEVATOR_PORT);
	}

	/**
	 * Sets elevator motors to speed
	 * 
	 * @param speed
	 */
	public void setElevatorSpeed(double speed) {
		leftElevator.set(-speed);
		rightElevator.set(speed);
	}

	/**
	 * Stalls elevator motors
	 * 
	 */
	public void stallElevator() {
		leftElevator.set(0.1);
		rightElevator.set(-0.1);
	}

	/**
	 * Stops elevator motors
	 */
	public void stopElevator() {
		leftElevator.set(0);
		rightElevator.set(0);
	}

	/**
	 * Switch from elevator mode to lifter mode
	 */
	public void switchElevatorPiston() {
		if (!elevatorPiston.get()) {
			elevatorPiston.set(true);
		} else {
			elevatorPiston.set(false);
		}
	}

	/**
	 * Get the current mode of the elevator piston
	 * 
	 * @return mode
	 */
	public String getMode() {
		if (elevatorPiston.get()) {
			mode = "Elevator";
			return mode;
		}
		mode = "Lifter";
		return mode;
	}

	/**
	 * Get current value of low switch
	 * 
	 * @return low position switch
	 */
	public boolean getLowSwitch() {
		return !lowPos.get();
	}

	/**
	 * Get current value of middle switch
	 * 
	 * @return middle position switch
	 */
	public boolean getMidSwitch() {
		return !midPos.get();
	}

	/**
	 * Get current value of high switch
	 * 
	 * @return high position switch
	 */
	public boolean getHighSwitch() {
		return !highPos.get();
	}

	/**
	 * Get current position of the elevator
	 * 
	 * @return elevator position
	 */
	public double getPosition() {
		if (getLowSwitch()) {
			position = 1;
		} else if (getMidSwitch()) {
			position = 2;
		} else if (getHighSwitch()) {
			position = 3;
		}
		return position;
	}

	/**
	 * Sets command to run via joysticks automatically
	 */
	@Override
	public void initDefaultCommand() {
		// Do nothing
	}
}
