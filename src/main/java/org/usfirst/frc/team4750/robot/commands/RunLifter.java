package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.OI;
import org.usfirst.frc.team4750.robot.Robot;
import org.usfirst.frc.team4750.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunLifter extends Command {

    public RunLifter() {
        requires(Robot.lifter);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	if(OI.controlDriveStick.getY() == -1) {
    		if(!Robot.elevator.getLowSwitch()) {
        		Robot.lifter.setLifterSpeed(RobotMap.LIFTER_SPEED);	
    		}else {
        		Robot.lifter.stopLifter();
    		}
    	}else {
    		Robot.lifter.stopLifter();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.lifter.stopLifter();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
