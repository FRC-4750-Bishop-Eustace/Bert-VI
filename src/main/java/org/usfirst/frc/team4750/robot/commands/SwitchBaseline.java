package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchBaseline extends CommandGroup {

	public SwitchBaseline() {
		addSequential(new DriveToDistance(10, true));
	}
}
