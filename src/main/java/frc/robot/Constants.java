// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
	public static double driveDirection = 1.0;
	public static double JOYSTICK_X_DEADZONE = 0.2;
	public static double JOYSTICK_Y_DEADZONE = 0.2;
	public static double JOYSTICK_Z_DEADZONE = 0.2;
	public static double JOYSTICK_Z2_DEADZONE = 0.2;

	public static double MAX_SPEED_JOYSTICK = 0.7;

	public static boolean FieldCentricDrive = true;
	
	public static double RobotTargetHeading = 0.0;
	public static boolean EnableAutoRobotHeading = false;

	// joy stick values
	public static double  OperatorLeftTrigger = 0.;
	public static double  OperatorRightTrigger = 0.;
	public static double  DriverLeftTrigger = 0.;
	public static double  DriverRightTrigger = 0.;
	public static int     OperatorPOV = -1;
	public static int     DriverPOV = -1;
	public static boolean Driverbackbutton = false;
	public static double  OperatorArmJoystick = 0.0;
	public static double  OperatorClimberJoystick = 0.0;
	public static double  DriverYaxis = 0.;
	public static double  DriverXaxis = 0.;
	public static boolean Driverstartbutton = false;

	public static int driverStick = 0;

	////////////////////////////////////////
	//               Swerve               //
	////////////////////////////////////////

	public static final double[] SWERVE_SETPOINT_OFFSET = { 
		//must be between 0 & 360 degrees
		49.31, //Front Right
		97.03, //Rear Right
		89.21, //Rear Left
		358.59  //Front Left
	}; 
       
	public static double[][] SWERVE_STEER_PID_CONSTANTS = { 
		// kP   kI   kD
		{ 0.8, 0.0, 0.016 }, //Front Right
		{ 0.8, 0.0, 0.016 }, //Rear Right
		{ 0.8, 0.0, 0.016 }, //Rear Left
		{ 0.8, 0.0, 0.016 }  //Front Left
	};

	//TODO: Add PID loop for drive motors
	public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Front Right
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Rear Right
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Rear Left
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }  //Front Left
	};

	/*
	 * Swerve rotation PID Constants
	 */
	public static double[] SWERVE_ROTATION_PID_CONSTANTS = {
		1.0,
		0.0,
		0.0
	};

	public static boolean[] STEER_MOTOR_INVERTED = { false, false, false, false };
	public static boolean[] DRIVE_MOTOR_INVERTED = { false, false, true, false };
	
	/*
	 * Swerve constants for swerve module calculations
	 * Don't question and just assume
	 * Want an explanation? Me too...
	 */
	public static double SWERVE_FRAME_LENGTH = 27.5;
	public static double SWERVE_FRAME_WIDTH = 27.5;
	public static double SWERVE_RADIUS = Math.sqrt( Math.pow( SWERVE_FRAME_LENGTH, 2 ) + Math.pow( SWERVE_FRAME_WIDTH, 2 ) );
	public static double SWERVE_PID_TOLERANCE = 2.8e-4;

	/*
	 * Arm IDs
	 */
	public static int SHOULDER_ID = 1;
	public static int EXTEND_ID = 1;

	/*
	 * Arm Constants
	 */
	public static double EXTEND_SPEED = 0.2;
	public static double[] ARM_ANGLE_PID_CONSTANTS = { 0.0, 0.0, 0.0 };

	/*
	 * Pnuematic IDs
	 */
	public static int CLAW_ID_OPEN = 1;
	public static int CLAW_ID_CLOSE = 2;
	public static int ARM_ID_OPEN = 3;
	public static int ARM_ID_CLOSE = 4;

	/*
	 * Swerve module motor and encoder ids
	 * { Front Right, Back Right, Back Left, Front Left }
	 */
	public static int[] SWERVE_DRIVE_MOTOR_IDS =     { 10, 11, 12, 13 };
	public static int[] SWERVE_STEER_MOTOR_IDS =     { 20, 21, 22, 23 };
	public static int[] SWERVE_ENCODER_IDS =         { 30, 31, 32, 33 };

	public static int swerveModuleNumber = 4;
}
