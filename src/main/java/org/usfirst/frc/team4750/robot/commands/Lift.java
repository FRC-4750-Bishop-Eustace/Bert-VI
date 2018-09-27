package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Lift extends CommandGroup {

    public Lift() {
        addSequential(new DriveToRaise());
        addSequential(new GoToHighLift());
        addSequential(new DriveToLift());
        addSequential(new GoToMidLift());
        addSequential(new SwitchElevatorMode());
    }
}
