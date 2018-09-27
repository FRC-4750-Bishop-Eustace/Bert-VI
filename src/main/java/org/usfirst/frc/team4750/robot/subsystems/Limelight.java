package org.usfirst.frc.team4750.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class handles the interfacing for the limelight camera
 */
public class Limelight extends Subsystem {

	// Create network table
	NetworkTable table;

	// Create variables
	boolean hasTarget;
	double xOffset, yOffset, area, skew, ledMode, camMode, pipeline;

	public Limelight() {
		// Instantiate limelight network table
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	/**
	 * Returns whether the target is in view or not
	 * 
	 * @return target is in view
	 */
	public boolean getHasTarget() {
		double targetD = table.getEntry("tv").getDouble(0);
		if (targetD == 0) {
			hasTarget = false;
		} else if (targetD == 1) {
			hasTarget = true;
		}
		return hasTarget;
	}

	/**
	 * Returns the xOffset from the target
	 * 
	 * @return xOffset from target
	 */
	public double getXOffset() {
		xOffset = table.getEntry("tx").getDouble(0);
		return xOffset;
	}

	/**
	 * Returns the yOffset from the target
	 * 
	 * @return yOffset from target
	 */
	public double getYOffset() {
		yOffset = table.getEntry("ty").getDouble(0);
		return yOffset;
	}

	/**
	 * Returns the area of the target
	 * 
	 * @return area of target
	 */
	public double getArea() {
		area = table.getEntry("ta").getDouble(0);
		return area;
	}

	/**
	 * Returns the skew of the target
	 * 
	 * @return skew of target
	 */
	public double getSkew() {
		skew = table.getEntry("ts").getDouble(0);
		return skew;
	}

	/**
	 * Returns the current LED mode
	 * 
	 * @return current LED mode 0 = on, 1 = off, 2 = blink
	 */
	public double getLEDMode() {
		ledMode = table.getEntry("ledMode").getDouble(0);
		return ledMode;
	}

	/**
	 * Returns the current camera mode
	 * 
	 * @return current camera mode 0 = threshold, 1 = raw image
	 */
	public double getCamMode() {
		camMode = table.getEntry("camMode").getDouble(0);
		return camMode;
	}

	/**
	 * Returns the current pipeline
	 * 
	 * @return current pipeline
	 */
	public double getPipeline() {
		pipeline = table.getEntry("pipeline").getDouble(0);
		return pipeline;
	}

	/**
	 * Cycles the LED mode, skipping blink mode
	 * 
	 */
	public void switchLED() {
		if (getLEDMode() == 0) {
			table.getEntry("ledMode").setDouble(1);
		} else if (getLEDMode() == 1) {
			table.getEntry("ledMode").setDouble(0);
		} else if (getLEDMode() == 2) {
			table.getEntry("ledMode").setDouble(1);
		}
	}

	/**
	 * Cycles the camera mode
	 * 
	 */
	public void switchCamera() {
		if (getCamMode() == 0) {
			table.getEntry("camMode").setDouble(1);
		} else if (getCamMode() == 1) {
			table.getEntry("camMode").setDouble(0);
		}
	}

	/**
	 * Set camera mode to vision or camera mode
	 * 
	 */
	public void setCameraMode(String mode) {
		if (mode.equalsIgnoreCase("Vision")) {
			table.getEntry("camMode").setDouble(0);
			table.getEntry("ledMode").setDouble(1);
			setPipeline(1);
		} else if (mode.equalsIgnoreCase("Camera")) {
			table.getEntry("camMode").setDouble(1);
			table.getEntry("ledMode").setDouble(1);
			setPipeline(0);
		}
	}

	/**
	 * Changes the current pipeline
	 * 
	 */
	public void setPipeline(double pipeline) {
		table.getEntry("pipeline").setDouble(pipeline);
	}

	@Override
	public void initDefaultCommand() {
		// No default command
	}
}
