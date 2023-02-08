// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.drive.GyroSwerveDriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive();
  public final GyroReset gyroReset = new GyroReset( gyro );

  private final ExampleCommand m_autoCommand = new ExampleCommand( m_exampleSubsystem );

  Joystick driveStick = new Joystick( Constants.driverStick );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //*
    m_gyroSwerveDrive.setDefaultCommand(
      new GyroSwerveDriveCommand(
        () -> driveStick.getX(),
        () -> driveStick.getY(),
        () -> driveStick.getZ(),
        gyro,
        m_gyroSwerveDrive
      )
    );
    //*/

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton triggerGyroReset = new JoystickButton(driveStick, 3);
    triggerGyroReset.whenPressed(new GyroReset(gyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
