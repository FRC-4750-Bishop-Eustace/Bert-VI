package org.usfirst.frc.team4750.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class sets up the chooser for the smart dashboard to select the goal
 * position
 * 
 */
public class GoalChooser {

	// Create chooser
	SendableChooser<String> objectiveChooser = new SendableChooser<>();

	public GoalChooser() {
		// Add options
		objectiveChooser.addDefault("Front Left Switch", "fls");
		objectiveChooser.addObject("Front Right Switch", "frs");
		objectiveChooser.addObject("Left Switch", "ls");
		objectiveChooser.addObject("Right Switch", "rs");
		objectiveChooser.addObject("Back Left Switch", "bls");
		objectiveChooser.addObject("Back Right Switch", "brs");
		objectiveChooser.addObject("Front Left Scale", "flsc");
		objectiveChooser.addObject("Front Right Scale", "frsc");
		objectiveChooser.addObject("Left Scale", "ls");
		objectiveChooser.addObject("Right Scale", "rs");
	}

	/**
	 * Returns the chooser to output it to the smart dashboard
	 * 
	 * @return sendable chooser object
	 */
	public SendableChooser<String> getChooser() {
		return objectiveChooser;
	}

	/**
	 * Returns the currently selected option
	 * 
	 * @return string chosen from the dashboard
	 */
	public String getSelected() {
		return objectiveChooser.getSelected();
	}
}
