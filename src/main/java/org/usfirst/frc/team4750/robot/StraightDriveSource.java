package org.usfirst.frc.team4750.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * This class acts as a PIDSource that returns the average of both the left and
 * right encoders
 *
 */
public class StraightDriveSource implements PIDSource {

	// Create variable
	double average;

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		// Get the average of the left and right encoders
		average = (Robot.encoders.getLeftDistanceInches() + Robot.encoders.getRightDistanceInches()) / 2;
		return average;
	}

}
