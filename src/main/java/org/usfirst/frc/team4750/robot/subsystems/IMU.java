package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.commands.Output;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class is the interface to the IMU hardware
 * 
 */
public class IMU extends Subsystem implements PIDSource {

	// Create IMU
	public AHRS ahrs;

	// Cumulative turning heading
	public double cumulativeCommandedHeading = 0.0;

	public IMU() {
		try {
			// Initialize IMU
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (Exception e) {
			System.out.println("IMU failed to connect");
		}
	}

	/**
	 * Use this to get the pitch
	 * 
	 * @return the degrees - is forward + is back
	 */
	public float getPitch() {
		return ahrs.getPitch();
	}

	/**
	 * Use this to get the angle
	 * 
	 * @return the degrees - is left + is right
	 */
	public float getAngle() {
		return (float) ahrs.getAngle();
	}

	/**
	 * Use this to get the current heading
	 * 
	 * @return the degrees - is left + is right
	 */
	public float getHeading() {
		return ahrs.getYaw();
	}

	/**
	 * This resets the IMU
	 * 
	 */
	public void reset() {
		ahrs.reset();
	}

	/**
	 * This sets the cumulative commanded heading
	 * 
	 */
	public void setCumulativeHeading(double heading) {
		cumulativeCommandedHeading += heading;
	}

	/**
	 * This returns the cumulative commanded heading
	 * 
	 * @return cumulative heading
	 */
	public double getCumulativeHeading() {
		return cumulativeCommandedHeading;
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new Output());
	}

	/**
	 * PIDSource required methods
	 * 
	 */
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kRate;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return 0;
	}
}
