package org.usfirst.frc.team4750.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class sets up the chooser for the smart dashboard to select the goal
 * position
 * 
 */
public class OrientationChooser {

	// Create chooser
	SendableChooser<String> orientationChooser = new SendableChooser<>();

	public OrientationChooser() {
		// Add options
		orientationChooser.addDefault("Forward", "f");
		orientationChooser.addObject("Left", "l");
		orientationChooser.addObject("Right", "r");
		orientationChooser.addObject("Back", "b");
	}

	/**
	 * Returns the chooser to output it to the smart dashboard
	 * 
	 * @return sendable chooser object
	 */
	public SendableChooser<String> getChooser() {
		return orientationChooser;
	}

	/**
	 * Returns the currently selected option
	 * 
	 * @return string chosen from the dashboard
	 */
	public String getSelected() {
		return orientationChooser.getSelected();
	}
}
