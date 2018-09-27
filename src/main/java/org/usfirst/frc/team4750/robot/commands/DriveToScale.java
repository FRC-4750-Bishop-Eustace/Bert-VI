package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveToScale extends CommandGroup {

	public DriveToScale() {
		addSequential(new DriveToDistance(24, true));
	}
}
