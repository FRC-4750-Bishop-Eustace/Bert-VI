package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightBaseline extends CommandGroup {

	public RightBaseline() {
		addParallel(new RotateToBottom());
		addParallel(new GoToMidPos());
		addSequential(new DriveToDistance(229, false));
		addSequential(new TurnToAngle(-90));
		addSequential(new DriveToDistance(4, true));
	}
}
