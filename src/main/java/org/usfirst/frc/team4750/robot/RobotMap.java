package org.usfirst.frc.team4750.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Joystick ports
	public static final int LEFT_JOYSTICK_PORT = 0;
	public static final int RIGHT_JOYSTICK_PORT = 1;
	public static final int CONTROL_JOYSTICK_PORT = 2;

	// Drive motor IDs (CAN ID)
	public static final int FRONT_LEFT_MOTOR_ID = 1;
	public static final int FRONT_RIGHT_MOTOR_ID = 2;
	public static final int LEFT_MOTOR_ID = 3;
	public static final int RIGHT_MOTOR_ID = 4;
	public static final int BACK_LEFT_MOTOR_ID = 5;
	public static final int BACK_RIGHT_MOTOR_ID = 6;

	// Digital Ports (DIO)
	public static final int ULTRASONIC_ECHO = 0;
	public static final int ULTRASONIC_TRIGGER = 1;
	public static final int LOW_POSITION_ELEVATOR_PORT = 2;
	public static final int MID_POSITION_ELEVATOR_PORT = 3;
	public static final int HIGH_POSITION_ELEVATOR_PORT = 4;
	public static final int LEFT_ENCODER_A = 5;
	public static final int LEFT_ENCODER_B = 6;
	public static final int RIGHT_ENCODER_A = 7;
	public static final int RIGHT_ENCODER_B = 8;
	public static final int LOW_POSITION_GRABBER_PORT = 23;
	public static final int HIGH_POSITION_GRABBER_PORT = 19;

	// Grabber ports (PWM/CAN ID)
	public static final int LEFT_ARM_MOTOR_PORT = 0;
	public static final int RIGHT_ARM_MOTOR_PORT = 1;
	public static final int ROTATE_MOTOR_ID = 8;

	// Grabber piston ports (PCM)
	public static final int GRABBER_PISTON_PORT = 1;

	// Elevator motor ports/IDs (PWM/CAN ID)
	public static final int LEFT_ELEVATOR_MOTOR_PORT = 2;
	public static final int RIGHT_ELEVATOR_MOTOR_PORT = 3;
	public static final int LIFTER_MOTOR_ID = 7;

	// Elevator piston ports (PCM)
	public static final int ELEVATOR_PISTON_PORT = 0;
	public static final int RELEASE_PISTON_PORT = 2;
	
	// Speeds
	public static final double ELEVATOR_UP_SPEED = -0.8;
	public static final double ELEVATOR_DOWN_SPEED = 0.4;
	public static final double ELEVATOR_DOWN_TO_MID_SPEED = 0.3;
	public static final double LIFTER_SPEED = 0.1;
	public static final double GRABBER_UP_SPEED = 0.6;
	public static final double GRABBER_DOWN_SPEED = -0.6;
}
