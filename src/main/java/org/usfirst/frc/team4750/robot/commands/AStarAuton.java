package org.usfirst.frc.team4750.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * This class creates the command group for the AStar path planning
 *
 */
public class AStarAuton extends CommandGroup {

	public AStarAuton(ArrayList<Command> drivePath) {
		for (Command c : drivePath) {
    		if(c.equals(new GoToHighPos()) || c.equals(new GoToMidPos()) || c.equals(new RotateToBottom())) {
    			addParallel(c);
    		}else {
        		addSequential(c);	
    		}
			System.out.print(c + " ran!");
		}
	}
}
