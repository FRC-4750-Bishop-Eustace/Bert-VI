package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LScaleFromL extends CommandGroup {

    public LScaleFromL() {
        addParallel(new RotateToBottom());
        addParallel(new GoToMidPos());
        addSequential(new DriveToDistance(300, false));
        addSequential(new TurnToAngle(90));
        addSequential(new DriveToDistance(-1.5f, true));
        addParallel(new GoToHighPos());
        addParallel(new RotateGrabberUp(true));
        addSequential(new DriveToDistance(1, true));
        addSequential(new RunGrabber(1, 0, true));
        addSequential(new SwitchGrabberMode());
        addSequential(new DriveToDistance(-1, true));
        addSequential(new TurnToAngle(-90));
        addParallel(new GoToLowPos());
        addParallel(new SwitchGrabberMode());
    }
}
