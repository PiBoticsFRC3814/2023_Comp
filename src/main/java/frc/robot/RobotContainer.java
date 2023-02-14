// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ADIS16470_IMU m_gyrp = new ADIS16470_IMU();

  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive();
  public final Arm m_arm = new Arm();
  public final Grabber m_grabber = new Grabber();

  private final AutoPosition m_followAprilTag = new AutoPosition(m_gyroSwerveDrive);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  XboxController driveController = new XboxController(2);
  XboxController armController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // *
    m_gyroSwerveDrive.setDefaultCommand(
        new GyroSwerveDriveCommand(
            () -> driveController.getLeftX(),
            () -> driveController.getLeftY(),
            () -> driveController.getRawAxis(2),
            () -> driveController.getRawAxis(3),
            m_gyrp,
            m_gyroSwerveDrive));

    m_arm.setDefaultCommand(
        new DirectArmCommand(
            m_arm, () -> armController.getLeftY(), () -> armController.getLeftX()));
    // */

    // Configure the button bindings
    configureButtonBindings();

    m_autoChooser.setDefaultOption("Follow Apriltag", m_followAprilTag);
    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveController, XboxController.Button.kY.value)
        .whileTrue(new GyroReset(m_gyrp));
    new JoystickButton(driveController, XboxController.Button.kB.value)
        .whileTrue(new HardBrake(m_gyroSwerveDrive));
    new JoystickButton(armController, XboxController.Button.kX.value)
        .whileTrue(new GrabberToggle(m_grabber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}
