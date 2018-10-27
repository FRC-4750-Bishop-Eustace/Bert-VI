package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Lift extends CommandGroup {

    public Lift() {
        addSequential(new RotateToTop());
        addSequential(new Release());
        addSequential(new DriveToRaise());
        addSequential(new GoToHighPos());
        addSequential(new DriveToLift());
        addSequential(new RotateGrabberDown(true));
        addSequential(new GoToMidPos());
        addSequential(new SwitchElevatorMode());
        addSequential(new GoToLowLift());
    }
}
