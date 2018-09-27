package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LFromL extends CommandGroup {

    public LFromL() {
        addParallel(new RotateToBottom());
        addParallel(new GoToMidPos());
        addSequential(new DriveToDistance(149, false));
        addSequential(new TurnToAngle(90));
        addSequential(new DriveToSwitch());
        addSequential(new RunGrabber(1, 0, true));
        addSequential(new SwitchGrabberMode());
        addSequential(new DriveToDistance(-2, true));
        addParallel(new GoToLowPos());
        addSequential(new TurnToAngle(-90));
    }
}
