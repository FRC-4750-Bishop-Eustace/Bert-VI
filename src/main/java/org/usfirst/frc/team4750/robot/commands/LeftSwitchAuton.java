package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftSwitchAuton extends CommandGroup {

	public LeftSwitchAuton() {
		addParallel(new RotateToBottom());
		addSequential(new DriveToDistance(13, true));
		addSequential(new TurnToAngle(90));
		addParallel(new GoToMidPos());
		addSequential(new DriveToSwitch());
		addSequential(new RunGrabber(1, 0, true));
		addParallel(new GoToLowPos());
		addSequential(new DriveToDistance(-2, true));
	}
}
