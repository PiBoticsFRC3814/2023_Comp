// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  ////////////////////////////////////////
  //                 OI                 //
  ////////////////////////////////////////

  public static final double JOYSTICK_X_DEADZONE = 0.2;
  public static final double JOYSTICK_Y_DEADZONE = 0.2;
  public static final double JOYSTICK_Z_DEADZONE = 0.2;
  public static final double JOYSTICK_Z2_DEADZONE = 0.2;

  public static final int DRIVE_CONTROLLER_PORT = 2;
  public static final int STEER_CONTROLLER_PORT = 0;
  public static final double SLOW_SPEED = 0.5;
  public static final double FAST_SPEED = 1.0;

  ////////////////////////////////////////
  //               Swerve               //
  ////////////////////////////////////////

  /*
   * Swerve module motor and encoder ids
   * { Front Right, Back Right, Back Left, Front Left }
   */
  public static final int[] SWERVE_DRIVE_MOTOR_IDS = {10, 11, 12, 13};
  public static final int[] SWERVE_STEER_MOTOR_IDS = {20, 21, 22, 23};
  public static final int[] SWERVE_ENCODER_IDS = {30, 31, 32, 33};

  public static final int swerveModuleNumber = 4;

  public static final double[] SWERVE_SETPOINT_OFFSET = {
    // must be between 0 & 360 degrees
    51.31, // Front Right
    97.03, // Rear Right
    84.21, // Rear Left
    355.59 // Front Left
  };

  public static final double[][] SWERVE_STEER_PID_CONSTANTS = {
    // kP   kI   kD
    {1.0, 0.0, 0.0}, // Front Right
    {1.0, 0.0, 0.0}, // Rear Right
    {1.0, 0.0, 0.0}, // Rear Left
    {1.0, 0.0, 0.0} // Front Left
  };

  /*
   * Swerve rotation PID Constants
   */
  public static final double[] SWERVE_ROTATION_PID_CONSTANTS = {1.0, 0.0, 0.0};

  public static final boolean[] STEER_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, false, true, false};

  /*
   * Swerve constants for swerve module calculations
   */
  public static final double SWERVE_FRAME_LENGTH = 27.5;
  public static final double SWERVE_FRAME_WIDTH = 27.5;
  public static final double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_FRAME_LENGTH, 2) + Math.pow(SWERVE_FRAME_WIDTH, 2));
  public static final double SWERVE_PID_TOLERANCE = 2.8e-4;

  ////////////////////////////////////////
  //             Arm & Claw             //
  ////////////////////////////////////////

	/*
	 * Arm Motor IDs
	 */
	public static final int SHOULDER_ID_1 = 40;
	public static final int SHOULDER_ID_2 = 41;
	public static final int EXTEND_ID = 42;

  /*
   * Arm control constants
   */
  public static final double EXTEND_HOME_SPEED = -0.6;
  public static final double[] EXTEND_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double[] ARM_ANGLE_PID_CONSTANTS = {5.0, 0.0, 0.1};
  public static final int ARM_ENCODER_PORT = 5;

  public static final double SCORE_DISTANCE = 1.0;
  public static final double SCORE_SIDE_TIME = 1.0;
  public static final double SCORE_SPEED = -0.2;

  public static final double SCORE_ANGLE_TOP = 0.595; // cone
  public static final double SCORE_ANGLE_MIDDLE = 0.57;
  public static final double SCORE_ANGLE_BOTTOM = 0.41;
  //todo: add logic for lower cube top:0.547; middle:0.500
  public static final double DEPLOY_ANGLE = 0.41;
  public static final double STOW_ANGLE = 0.375;
  public static final double SUBSTATION_ANGLE = 20.0;
  public static final double EXTEND_REVS_1 = -13.09;
  public static final double EXTEND_REVS_2 = -88.45;
  public static final double EXTEND_REVS_3 = -145.88;

  /*
   * Claw and Arm Pnuematic IDs
   */
  public static final int CLAW_ID_OPEN = 0;
  public static final int CLAW_ID_CLOSE = 1;
  public static final int ARM_ID_OPEN = 3;
  public static final int ARM_ID_CLOSE = 2;
}
