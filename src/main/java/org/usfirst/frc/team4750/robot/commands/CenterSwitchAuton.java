package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterSwitchAuton extends CommandGroup {

	public CenterSwitchAuton() {
		addParallel(new RotateToBottom());
		addParallel(new GoToMidPos());
		addSequential(new DriveToDistance(140, false));
		addSequential(new DriveToSwitch());
		addSequential(new RunGrabber(1, 0, true));
		addParallel(new GoToLowPos());
		addSequential(new DriveToDistance(-2, true));
	}
}
