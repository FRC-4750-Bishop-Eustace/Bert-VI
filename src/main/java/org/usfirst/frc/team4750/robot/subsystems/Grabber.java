package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This subsystem handles the motors and piston of the cube grabber mechanism
 *
 */
public class Grabber extends Subsystem {

	// Motors
	VictorSP leftArm;
	VictorSP rightArm;
	TalonSRX rotate;

	// Arm drive
	DifferentialDrive armDrive;

	// Pistons
	Solenoid piston;

	// Limit switches
	DigitalInput low;
	DigitalInput high;

	public Grabber() {
		// Initialize motors
		leftArm = new VictorSP(RobotMap.LEFT_ARM_MOTOR_PORT);
		rightArm = new VictorSP(RobotMap.RIGHT_ARM_MOTOR_PORT);
		rotate = new TalonSRX(RobotMap.ROTATE_MOTOR_ID);

		// Initialize arm drive
		armDrive = new DifferentialDrive(leftArm, rightArm);

		armDrive.setSafetyEnabled(false);

		// Initialize pistons
		piston = new Solenoid(RobotMap.GRABBER_PISTON_PORT);

		// Initialize limit switches
		low = new DigitalInput(RobotMap.LOW_POSITION_GRABBER_PORT);
		high = new DigitalInput(RobotMap.HIGH_POSITION_GRABBER_PORT);
	}

	/**
	 * This method opens or closes the grabber's arms
	 */
	public void switchPistons() {
		if (piston.get()) {
			piston.set(false);
		} else {
			piston.set(true);
		}
	}

	/**
	 * This method drives the arm motors
	 * 
	 * @param speed
	 * @param rotateSpeed
	 */
	public void armDrive(double speed, double rotateSpeed) {
		armDrive.arcadeDrive(speed, rotateSpeed);
	}

	/**
	 * This method rotates the grabber up and down
	 * 
	 * @param speed
	 */
	public void rotateGrabber(double speed) {
		rotate.set(ControlMode.PercentOutput, speed);
	}

	/**
	 * This returns the current value of the low limit switch
	 * 
	 * @return low switch
	 */
	public boolean getLow() {
		return low.get();
	}

	/**
	 * This returns the current value of the high limit switch
	 * 
	 * @return high switch
	 */
	public boolean getHigh() {
		return high.get();
	}

	@Override
	public void initDefaultCommand() {
		// Do nothing
	}
}
