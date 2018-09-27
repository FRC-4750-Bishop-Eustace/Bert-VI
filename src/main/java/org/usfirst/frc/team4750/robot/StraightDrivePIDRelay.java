package org.usfirst.frc.team4750.robot;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * This class acts as a relay to allow a second PIDController to control the
 * drive train
 *
 */
public class StraightDrivePIDRelay implements PIDOutput {

	@Override
	public void pidWrite(double output) {
		Robot.driveTrain.pidStraightWrite(output);
	}
}
