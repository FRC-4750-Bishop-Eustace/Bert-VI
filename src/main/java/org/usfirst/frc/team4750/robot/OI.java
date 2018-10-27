package org.usfirst.frc.team4750.robot;

import org.usfirst.frc.team4750.robot.commands.GoToHighPos;
import org.usfirst.frc.team4750.robot.commands.GoToLowPos;
import org.usfirst.frc.team4750.robot.commands.GoToMidPos;
import org.usfirst.frc.team4750.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team4750.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team4750.robot.commands.RunGrabber;
import org.usfirst.frc.team4750.robot.commands.SwitchGrabberMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	// Joysticks
	public static Joystick leftDriveStick = new Joystick(RobotMap.LEFT_JOYSTICK_PORT);
	public static Joystick rightDriveStick = new Joystick(RobotMap.RIGHT_JOYSTICK_PORT);
	public static Joystick controlDriveStick = new Joystick(RobotMap.CONTROL_JOYSTICK_PORT);

	// Elevator buttons
	Button elevatorTop = new JoystickButton(controlDriveStick, 5);
	Button elevatorMid= new JoystickButton(controlDriveStick, 3);
	Button elevatorLow = new JoystickButton(controlDriveStick, 1);

	// Grabber buttons
	Button grabberSwitch = new JoystickButton(controlDriveStick, 8);
	Button grabberOut = new JoystickButton(controlDriveStick, 9);
	Button grabberIn = new JoystickButton(controlDriveStick, 6);
	Button grabberLeft = new JoystickButton(controlDriveStick, 10);
	Button grabberRight = new JoystickButton(controlDriveStick, 7);
	Button grabberRotateDown = new JoystickButton(controlDriveStick, 2);
	Button grabberRotateUp = new JoystickButton(controlDriveStick, 4);

	public OI() {
		// Elevator buttons
		elevatorTop.whenReleased(new GoToHighPos());
		elevatorMid.whenReleased(new GoToMidPos());
		elevatorLow.whenReleased(new GoToLowPos());

		// Grabber buttons
		grabberSwitch.whenReleased(new SwitchGrabberMode());
		grabberOut.whileHeld(new RunGrabber(1, 0, false));
		grabberIn.whileHeld(new RunGrabber(-1, 0, false));
		grabberLeft.whileHeld(new RunGrabber(0, 1, false));
		grabberRight.whileHeld(new RunGrabber(0, -1, false));
		grabberRotateDown.whileHeld(new RotateGrabberDown(false));
		grabberRotateUp.whileHeld(new RotateGrabberUp(false));
	}
}
